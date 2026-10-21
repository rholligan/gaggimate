#include <unity.h>

#include "VirtualGaggiaPlant.hpp"
#include <control_physics/AutoCalibrator.hpp>
#include <control_physics/ComplianceModel.hpp>
#include <control_physics/FlashSteamEngine.hpp>
#include <control_physics/PumpSurface.hpp>
#include <control_physics/ThermalObserver.hpp>

using namespace control_physics;
using namespace test_sim;

void setUp(void) {}
void tearDown(void) {}

void test_thermal_safety_shutoff_on_overheat(void) {
    ThermalObserver observer(0.02f);

    // Normal operation
    float duty = observer.update(93.0f, 93.0f, 0.0f);
    TEST_ASSERT_FALSE(observer.isEmergencyTripped());

    // Overheat threshold: 155 deg C
    duty = observer.update(155.5f, 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // NaN / Inf fault protection
    duty = observer.update(std::numeric_limits<float>::quiet_NaN(), 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());
}

void test_zero_lag_flow_feedforward(void) {
    ThermalObserver observer(0.02f, 1490.0f);
    observer.setInletTemp(18.0f);
    observer.setEfficiency(1.0f);

    // Q = 2.5 ml/s, Target = 93 deg C, Tin = 18 deg C -> deltaT = 75 deg C
    // P_ff = 4.184 * 2.5 * 75 = 784.5 Watts
    // Expected duty = 784.5 / 1490.0 = ~0.5265
    const float duty = observer.computeFeedForwardDuty(2.5f, 93.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5265f, duty);
}

void test_smith_predictor_stability_with_deadtime(void) {
    VirtualGaggiaPlant plant(0.02f);
    ThermalObserver observer(0.02f, VirtualGaggiaPlant::kPmax);

    // 1. Initial warmup with cold group (testing dynamic warmup bias delivering target to puck)
    plant.reset(85.0f, 80.0f);
    observer.reset(85.0f, 80.0f);

    for (int step = 0; step < 1250; ++step) {
        const float T_sens = plant.getSensorTemp();
        const float ssrDuty = observer.update(T_sens, 93.0f, 0.0f);
        plant.step(ssrDuty, 0.0f, false);
    }

    // Dynamic warmup delivers target temp to the puck (estimated puck temp ~ 93 deg C)
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 93.0f, observer.estimatePuckTemperature());

    // 2. Fully soaked thermal equilibrium (boiler == group == 93 deg C)
    plant.reset(93.0f, 93.0f);
    observer.reset(93.0f, 93.0f);
    for (int step = 0; step < 500; ++step) {
        const float T_sens = plant.getSensorTemp();
        const float ssrDuty = observer.update(T_sens, 93.0f, 0.0f);
        plant.step(ssrDuty, 0.0f, false);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 93.0f, plant.getBoilerTemp());
}

void test_pump_surface_convexity_and_thermal_fade(void) {
    PumpSurface pump(0.02f);

    // 1. Monotonic flow scaling with duty at 9 bar
    const float flowLow = pump.predictFlow(9.0f, 0.50f);
    const float flowHigh = pump.predictFlow(9.0f, 0.85f);
    TEST_ASSERT_GREATER_THAN_FLOAT(flowLow, flowHigh);

    // 2. Monotonic bisection inversion
    const float commandedDuty = pump.invertDuty(2.5f, 9.0f);
    const float achievedFlow = pump.predictFlow(9.0f, commandedDuty);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.5f, achievedFlow);

    // 3. Solenoid coil thermal fade
    // Run 60 seconds at 100% duty
    for (int i = 0; i < 3000; ++i) {
        pump.updateThermalCoil(1.0f);
    }
    const float alpha = pump.getCoilAlpha();
    // Coil should show 10-15% fade (alpha ~ 0.85-0.90)
    TEST_ASSERT_TRUE(alpha < 0.95f && alpha >= 0.80f);
}

void test_compliance_observer_isolates_puck_flow(void) {
    ComplianceModel compliance(0.02f);

    // Step 1: Rapid pressure rise from 0 to 8 bar over 1 second (dP/dt = 8 bar/s)
    // Pump flow = 6 ml/s. System compliance absorbs water; Q_puck should be significantly lower than Q_raw
    float lastQpuck = 0.0f;
    for (int i = 0; i < 50; ++i) {
        const float P = 8.0f * (static_cast<float>(i + 1) / 50.0f);
        lastQpuck = compliance.update(P, 6.0f);
    }

    // Compliance flux C_sys * dP/dt should subtract significant flow from Q_raw
    TEST_ASSERT_TRUE(lastQpuck < 5.5f);
    TEST_ASSERT_TRUE(lastQpuck > 0.0f);
}

void test_darcy_channeling_auto_save_trigger(void) {
    VirtualGaggiaPlant plant(0.02f);
    ComplianceModel compliance(0.02f);

    plant.reset();
    compliance.reset();

    // 1. Establish 9 bar steady brew state (150 steps = 3.0s at 50Hz)
    for (int i = 0; i < 150; ++i) {
        plant.step(0.0f, 0.65f, true);
        compliance.update(plant.getPressure(), plant.getPumpFlow());
    }
    TEST_ASSERT_FALSE(compliance.isAutoSaveActive());

    // 2. Inject sudden channeling blowout in puck bed
    plant.injectChannelingFault();
    bool triggered = false;
    for (int i = 0; i < 150; ++i) {
        plant.step(0.0f, 0.65f, true);
        compliance.update(plant.getPressure(), plant.getPumpFlow());
        if (compliance.isAutoSaveActive()) {
            triggered = true;
            break;
        }
    }

    // Auto-Save must detect rapid resistance collapse and trigger flow ceiling
    TEST_ASSERT_TRUE(triggered);
}

void test_flash_steam_closed_loop_ceiling(void) {
    FlashSteamEngine steam(0.02f);
    steam.startSteamMode();

    float heaterDuty = 0.0f;
    float pumpTargetFlow = 0.0f;

    // Superheat state
    steam.update(120.0f, 1.0f, 0.0f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::SUPERHEAT), static_cast<int>(steam.getState()));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, heaterDuty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pumpTargetFlow);

    // Reached 148 deg C
    steam.update(148.0f, 3.5f, 0.0f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::READY), static_cast<int>(steam.getState()));

    // User cracks steam valve: dP/dt drops rapidly (-0.25 bar/s)
    steam.update(147.0f, 3.2f, -0.25f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::STEAMING_INJECTION), static_cast<int>(steam.getState()));

    // Pressure drops to 1.8 bar; injection active but capped at thermodynamic ceiling (0.518 ml/s)
    steam.update(144.0f, 1.8f, 0.0f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_TRUE(pumpTargetFlow <= FlashSteamEngine::kMaxInjectionRate);
    TEST_ASSERT_TRUE(pumpTargetFlow > 0.0f);
}

void test_autocalibrator_hydraulic_state_machine(void) {
    AutoCalibrator cal(0.02f);
    cal.startHydraulicCalibration();
    TEST_ASSERT_EQUAL(static_cast<int>(AutoCalibrator::HydraulicState::STAGE_A_OPEN_FLOW),
                      static_cast<int>(cal.getHydraulicState()));

    float pumpDuty = 0.0f;
    // Tick through 6 duty steps (4s each = 200 ticks each)
    for (size_t step = 0; step < AutoCalibrator::kNumDutySteps; ++step) {
        for (int i = 0; i < 201; ++i) {
            cal.updateHydraulic(0.0f, 2.0f * (step + 1), pumpDuty);
        }
    }

    // Transitions to blind compliance stage
    cal.transitionToBlindBasket();
    TEST_ASSERT_EQUAL(static_cast<int>(AutoCalibrator::HydraulicState::STAGE_B_BLIND_COMPLIANCE),
                      static_cast<int>(cal.getHydraulicState()));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_thermal_safety_shutoff_on_overheat);
    RUN_TEST(test_zero_lag_flow_feedforward);
    RUN_TEST(test_smith_predictor_stability_with_deadtime);
    RUN_TEST(test_pump_surface_convexity_and_thermal_fade);
    RUN_TEST(test_compliance_observer_isolates_puck_flow);
    RUN_TEST(test_darcy_channeling_auto_save_trigger);
    RUN_TEST(test_flash_steam_closed_loop_ceiling);
    RUN_TEST(test_autocalibrator_hydraulic_state_machine);
    return UNITY_END();
}
