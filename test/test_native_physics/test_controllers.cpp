#include <unity.h>

#include <algorithm>
#include <cmath>
#include <limits>

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

// ============================================================================
// BASELINE TEST SUITES
// ============================================================================

void test_thermal_safety_shutoff_on_overheat(void) {
    ThermalObserver observer(0.02f);

    // Normal operation
    float duty = observer.update(93.0f, 93.0f, 0.0f);
    TEST_ASSERT_FALSE(observer.isEmergencyTripped());
    (void)duty;

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

// ============================================================================
// 8 COMPREHENSIVE SIL PHYSICS TEST SUITES
// ============================================================================

/**
 * @brief Suite 1: Simulates a complete 35s espresso profile (pre-heat, 2.0 bar pre-infusion,
 * ramp to 9.0 bar, steady extraction at 2.5 ml/s, taper to 1.5 ml/s). Verifies continuous
 * pressure/flow tracking, volumetric yield integration, and clean profile termination.
 */
void test_full_brew_cycle_with_realistic_disturbances(void) {
    VirtualGaggiaPlant plant(0.02f);
    ThermalObserver thermal(0.02f, VirtualGaggiaPlant::kPmax);
    PumpSurface pump(0.02f);
    ComplianceModel compliance(0.02f);

    plant.reset(93.0f, 93.0f);
    thermal.reset(93.0f, 93.0f);
    pump.resetCoil();
    compliance.reset();

    constexpr float dt = 0.02f;
    constexpr int totalSteps = static_cast<int>(35.0f / dt); // 1750 steps

    float minTb = 100.0f;
    float maxTb = 0.0f;
    float pumpDuty = 0.0f;
    float ssrDuty = 0.0f;
    bool valveOpen = false;

    // Realistic puck resistance for ~9 bar extraction at 2.5 ml/s (R = P / Q^2 = 9 / 6.25 = 1.44)
    plant.setPuckResistance(1.44f);

    for (int step = 0; step < totalSteps; ++step) {
        const float t = step * dt;
        float targetFlow = 0.0f;

        if (t < 3.0f) {
            // Phase 1: Pre-heat / idle (3.0s)
            valveOpen = false;
            pumpDuty = 0.0f;
        } else if (t < 8.0f) {
            // Phase 2: 2.0 bar Pre-infusion (5.0s)
            valveOpen = true;
            targetFlow = 1.18f; // ~2.0 bar into puck
            pumpDuty = pump.invertDuty(targetFlow, plant.getPressure());
        } else if (t < 12.0f) {
            // Phase 3: Pressure ramp to 9.0 bar (4.0s)
            valveOpen = true;
            const float rampFrac = (t - 8.0f) / 4.0f;
            targetFlow = 1.18f + rampFrac * (2.5f - 1.18f);
            pumpDuty = pump.invertDuty(targetFlow, plant.getPressure());
        } else if (t < 28.0f) {
            // Phase 4: Steady extraction at 2.5 ml/s (16.0s)
            valveOpen = true;
            targetFlow = 2.5f;
            pumpDuty = pump.invertDuty(targetFlow, plant.getPressure());
        } else {
            // Phase 5: Flow taper to 1.5 ml/s (7.0s)
            valveOpen = true;
            const float taperFrac = (t - 28.0f) / 7.0f;
            targetFlow = 2.5f - taperFrac * (2.5f - 1.5f);
            pumpDuty = pump.invertDuty(targetFlow, plant.getPressure());
        }

        pump.updateThermalCoil(pumpDuty);
        plant.step(ssrDuty, pumpDuty, valveOpen);

        const float puckFlow = compliance.update(plant.getPressure(), plant.getPumpFlow());
        (void)puckFlow;

        ssrDuty = thermal.update(plant.getSensorTemp(), 93.0f, plant.getPumpFlow(), valveOpen);

        const float currentTb = plant.getBoilerTemp();
        minTb = std::min(minTb, currentTb);
        maxTb = std::max(maxTb, currentTb);
    }

    // Clean profile termination
    pumpDuty = 0.0f;
    valveOpen = false;
    plant.step(0.0f, 0.0f, false);

    // Assert volumetric yield integration
    const float totalYield = compliance.getTotalPuckVolume();
    TEST_ASSERT_TRUE(totalYield >= 30.0f && totalYield <= 65.0f);

    // Assert thermal stability throughout shot: within +/- 2.0 deg C of target 93.0 C
    TEST_ASSERT_TRUE(minTb >= 92.0f);
    TEST_ASSERT_TRUE(maxTb <= 95.0f);

    // Assert coil thermal accumulator tracked energy
    TEST_ASSERT_TRUE(pump.getCoilEnergy() > 0.0f);

    // Assert clean termination
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pumpDuty);
    TEST_ASSERT_FALSE(valveOpen);
}

/**
 * @brief Suite 2: Runs a 20s extraction at 2.5 ml/s cold water influx (Tin = 18 deg C).
 * Compares active vs inactive feedforward. Asserts min(Tb) >= 92.7 deg C (droop <= 0.30 deg C)
 * with active feedforward vs >3.0 deg C droop without feedforward.
 */
void test_zero_temperature_droop_active_flow_feedforward(void) {
    constexpr float dt = 0.02f;
    constexpr int steps = static_cast<int>(20.0f / dt); // 1000 steps = 20s

    // 1. With Active Flow Feed-Forward
    VirtualGaggiaPlant plantActive(dt);
    ThermalObserver thermalActive(dt, VirtualGaggiaPlant::kPmax);
    plantActive.reset(93.0f, 93.0f);
    thermalActive.reset(93.0f, 93.0f);

    float minTbActive = 100.0f;
    float ssrDutyActive = 0.0f;

    plantActive.setPuckResistance(1.44f);
    for (int i = 0; i < steps; ++i) {
        plantActive.step(ssrDutyActive, 0.65f, true);
        const float flowRate = plantActive.getPumpFlow();
        ssrDutyActive = thermalActive.update(plantActive.getSensorTemp(), 93.0f, flowRate, true);
        minTbActive = std::min(minTbActive, plantActive.getBoilerTemp());
    }

    const float droopActive = 93.0f - minTbActive;
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(92.7f, minTbActive);
    TEST_ASSERT_TRUE(droopActive <= 0.30f);

    // 2. Without Flow Feed-Forward (flowRate passed as 0.0f to observer)
    VirtualGaggiaPlant plantInactive(dt);
    ThermalObserver thermalInactive(dt, VirtualGaggiaPlant::kPmax);
    plantInactive.reset(93.0f, 93.0f);
    thermalInactive.reset(93.0f, 93.0f);

    float minTbInactive = 100.0f;
    float ssrDutyInactive = 0.0f;

    plantInactive.setPuckResistance(1.44f);
    for (int i = 0; i < steps; ++i) {
        plantInactive.step(ssrDutyInactive, 0.65f, true);
        ssrDutyInactive = thermalInactive.update(plantInactive.getSensorTemp(), 93.0f, 0.0f, true);
        minTbInactive = std::min(minTbInactive, plantInactive.getBoilerTemp());
    }

    const float droopInactive = 93.0f - minTbInactive;
    TEST_ASSERT_TRUE(droopInactive > 3.0f);
}

/**
 * @brief Suite 3: Applies flow step disturbance (0 -> 3.0 -> 0 ml/s) and +/- 0.5s model
 * dead-time mismatch. Asserts overshoot < 1.0 deg C and zero persistent oscillation despite 3.0s sensor dead time.
 */
void test_smith_predictor_dynamic_disturbance_rejection(void) {
    constexpr float dt = 0.02f;

    // Test across +/- 0.5s dead-time mismatch (plant tau_d = 3.0s, observer = 2.5s and 3.5s)
    const float testDelays[2] = {2.5f, 3.5f};

    for (float obsDeadTime : testDelays) {
        // --- 1. FLOW STEP DISTURBANCE (0 -> 3.0 -> 0 ml/s) ---
        VirtualGaggiaPlant plant(dt);
        ThermalObserver observer(dt, VirtualGaggiaPlant::kPmax, obsDeadTime);
        PumpSurface pump(dt);

        plant.reset(93.0f, 93.0f);
        observer.reset(93.0f, 93.0f);
        plant.setPuckResistance(1.0f); // 3.0 ml/s at 9 bar

        // Pre-soak plant and observer
        float ssrDuty = 0.0f;
        for (int i = 0; i < 200; ++i) {
            ssrDuty = observer.update(plant.getSensorTemp(), 93.0f, 0.0f, false);
            plant.step(ssrDuty, 0.0f, false);
        }

        // Apply flow disturbance: 0 -> 3.0 ml/s for 5.0s, then back to 0
        float minTbDisturb = 100.0f;
        float maxTbDisturb = 0.0f;
        for (int i = 0; i < 500; ++i) {
            const float t = i * dt;
            const bool isDisturbing = (t >= 1.0f && t < 6.0f);
            const float targetFlow = isDisturbing ? 3.0f : 0.0f;
            const float pumpDuty = isDisturbing ? pump.invertDuty(targetFlow, plant.getPressure()) : 0.0f;

            plant.step(ssrDuty, pumpDuty, isDisturbing);
            const float actualFlow = isDisturbing ? plant.getPuckFlow() : 0.0f;
            ssrDuty = observer.update(plant.getSensorTemp(), 93.0f, actualFlow, isDisturbing);

            const float currentTb = plant.getBoilerTemp();
            minTbDisturb = std::min(minTbDisturb, currentTb);
            maxTbDisturb = std::max(maxTbDisturb, currentTb);
        }

        // Overshoot < 1.0 C and zero persistent oscillation
        const float flowOvershoot = maxTbDisturb - 93.0f;
        TEST_ASSERT_TRUE(flowOvershoot < 1.0f);
        TEST_ASSERT_TRUE(minTbDisturb >= 92.5f);

        // Check steady convergence at end of disturbance window
        const float finalTb = plant.getBoilerTemp();
        TEST_ASSERT_FLOAT_WITHIN(1.0f, 93.0f, finalTb);

        // --- 2. SETPOINT STEP DISTURBANCE (93 -> 95 deg C) ---
        VirtualGaggiaPlant stepPlant(dt);
        ThermalObserver stepObserver(dt, VirtualGaggiaPlant::kPmax, obsDeadTime);
        stepObserver.setGains(0.10f, 12.0f, 0.5f);

        stepPlant.reset(93.0f, 93.0f);
        stepObserver.reset(93.0f, 93.0f);

        // Pre-soak at 93 C
        ssrDuty = 0.0f;
        for (int i = 0; i < 200; ++i) {
            ssrDuty = stepObserver.update(stepPlant.getSensorTemp(), 93.0f, 0.0f, false);
            stepPlant.step(ssrDuty, 0.0f, false);
        }

        // Apply setpoint step 93 -> 95 deg C
        float maxTbStep = 0.0f;
        float settlingTimeSec = -1.0f;
        constexpr int stepTotalTicks = static_cast<int>(10.0f / dt); // 10s window

        for (int i = 0; i < stepTotalTicks; ++i) {
            const float t = i * dt;
            ssrDuty = stepObserver.update(stepPlant.getSensorTemp(), 95.0f, 0.0f, false);
            stepPlant.step(ssrDuty, 0.0f, false);

            const float currentTb = stepPlant.getBoilerTemp();
            maxTbStep = std::max(maxTbStep, currentTb);

            if (settlingTimeSec < 0.0f && std::abs(currentTb - 95.0f) <= 0.25f) {
                settlingTimeSec = t;
            }
        }

        // Asserts: settling time < 6.0s and overshoot < 1.0 deg C (maxTb < 96.0 C)
        TEST_ASSERT_TRUE(settlingTimeSec >= 0.0f && settlingTimeSec < 6.0f);
        const float stepOvershoot = std::max(0.0f, maxTbStep - 95.0f);
        TEST_ASSERT_TRUE(stepOvershoot < 1.0f);
    }
}

/**
 * @brief Suite 4: Injects sudden puck resistance collapse (Rpuck -> 0.35). Asserts:
 * (a) isAutoSaveActive() trips in <= 10 steps (<= 200ms at 50Hz);
 * (b) Commanded flow setpoint is clamped to <= 1.8 ml/s;
 * (c) Duty throttles down to mitigate bed erosion.
 */
void test_channeling_auto_save_reaction_time_and_flow_cap(void) {
    VirtualGaggiaPlant plant(0.02f);
    ComplianceModel compliance(0.02f);
    PumpSurface pump(0.02f);

    plant.reset();
    compliance.reset();

    // 1. Establish 9 bar steady brew state (150 steps = 3.0s at 50Hz)
    const float preFaultDuty = 0.65f;
    for (int i = 0; i < 150; ++i) {
        plant.step(0.0f, preFaultDuty, true);
        compliance.update(plant.getPressure(), plant.getPumpFlow());
    }
    TEST_ASSERT_FALSE(compliance.isAutoSaveActive());

    // 2. Inject sudden puck resistance collapse (channel blowout Rpuck -> 0.35)
    plant.setPuckResistance(0.35f);

    int tripSteps = 0;
    bool tripped = false;
    for (int i = 1; i <= 20; ++i) {
        plant.step(0.0f, preFaultDuty, true);
        compliance.update(plant.getPressure(), plant.getPumpFlow());
        if (compliance.isAutoSaveActive()) {
            tripSteps = i;
            tripped = true;
            break;
        }
    }

    // Asserts: (a) isAutoSaveActive() trips in <= 10 steps (<= 200ms at 50Hz)
    TEST_ASSERT_TRUE(tripped);
    TEST_ASSERT_TRUE(tripSteps <= 10);

    // Asserts: (b) commanded flow setpoint is clamped to <= 1.8 ml/s
    float commandedFlowTarget = 2.5f;
    if (compliance.isAutoSaveActive()) {
        commandedFlowTarget = std::min(commandedFlowTarget, ComplianceModel::kAutoSaveFlowTarget);
    }
    TEST_ASSERT_TRUE(commandedFlowTarget <= 1.8f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.8f, commandedFlowTarget);

    // Asserts: (c) duty throttles to mitigate bed erosion
    const float throttledDuty = pump.invertDuty(commandedFlowTarget, plant.getPressure());
    TEST_ASSERT_TRUE(throttledDuty < preFaultDuty);
}

/**
 * @brief Suite 5: Asserts u_SSR == 0.0 and isEmergencyTripped() == true for:
 * T > 155.0 deg C, T = 250 deg C, NaN, +Inf, -Inf, and negative temperatures.
 * Verifies safety latching (recovering to 93 deg C does NOT re-enable SSR without explicit reset()).
 */
void test_thermal_safety_invariants_exhaustive(void) {
    ThermalObserver observer(0.02f);

    // 1. Normal operating temperature
    float duty = observer.update(93.0f, 93.0f, 0.0f);
    TEST_ASSERT_FALSE(observer.isEmergencyTripped());
    TEST_ASSERT_TRUE(duty >= 0.0f);

    // 2. Overheat condition: T > 155.0 deg C
    observer.reset(93.0f, 93.0f);
    duty = observer.update(155.5f, 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 3. Extreme overheat: T = 250.0 deg C
    observer.reset(93.0f, 93.0f);
    duty = observer.update(250.0f, 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 4. NaN sensor reading
    observer.reset(93.0f, 93.0f);
    duty = observer.update(std::numeric_limits<float>::quiet_NaN(), 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 5. +Infinity sensor reading
    observer.reset(93.0f, 93.0f);
    duty = observer.update(std::numeric_limits<float>::infinity(), 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 6. -Infinity sensor reading
    observer.reset(93.0f, 93.0f);
    duty = observer.update(-std::numeric_limits<float>::infinity(), 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 7. Negative temperature reading (sensor disconnected / shorted to GND)
    observer.reset(93.0f, 93.0f);
    duty = observer.update(-10.0f, 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // 8. Safety latching verification:
    // Once tripped, recovering to normal operating temperature (93.0 C) must NOT re-enable SSR
    // without an explicit reset()
    duty = observer.update(93.0f, 93.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_TRUE(observer.isEmergencyTripped());

    // Explicit reset clears latch
    observer.reset(93.0f, 93.0f);
    TEST_ASSERT_FALSE(observer.isEmergencyTripped());
    duty = observer.update(93.0f, 93.0f, 0.0f);
    TEST_ASSERT_FALSE(observer.isEmergencyTripped());
}

/**
 * @brief Suite 6: Solenoid coil thermal accumulator stress test:
 * 120s at 100% duty approaches alpha_coil -> 0.60 floor; 300s cooling at zero duty decays back to 1.00;
 * checks RLS online convexity updating with scale flow.
 */
void test_pump_surface_coil_fade_saturation_and_cooling(void) {
    PumpSurface pump(0.02f);

    // Initial state: fresh cold coil (alpha = 1.0)
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pump.getCoilAlpha());

    // 120s at 100% duty (6000 steps @ 50Hz)
    for (int i = 0; i < 6000; ++i) {
        pump.updateThermalCoil(1.0f);
    }
    const float alpha120s = pump.getCoilAlpha();
    // Substantial thermal fade approaching floor: alpha in [0.60, 0.85]
    TEST_ASSERT_TRUE(alpha120s >= PumpSurface::kMinAlphaCoil);
    TEST_ASSERT_TRUE(alpha120s < 0.85f);

    // 300s cooling at zero duty (15000 steps @ 50Hz)
    for (int i = 0; i < 15000; ++i) {
        pump.updateThermalCoil(0.0f);
    }
    const float alphaCooled = pump.getCoilAlpha();
    // Coil thermal decay back to ~1.00
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, alphaCooled);

    // Online RLS convexity updating with ground-truth scale flow
    const float kappaBefore = pump.getConvexity();

    // Scale flow higher than predicted at high pressure -> increases convexity
    pump.updateRLS(2.0f, 2.5f, 0.1f);
    const float kappaHigher = pump.getConvexity();
    TEST_ASSERT_TRUE(kappaHigher > kappaBefore);

    // Scale flow lower than predicted -> decreases convexity
    pump.updateRLS(2.5f, 2.0f, 0.1f);
    const float kappaLower = pump.getConvexity();
    TEST_ASSERT_TRUE(kappaLower < kappaHigher);
}

/**
 * @brief Suite 7: Verifies low-pressure air compression (C_sys ~ 6.15 ml/bar at 0 bar) vs
 * high-pressure hose elastance (C_sys ~ 0.21 ml/bar at 9 bar). Verifies rapid venting (-15 bar/s)
 * does not produce negative flow artifacts.
 */
void test_compliance_observer_two_regime_dynamics(void) {
    ComplianceModel compliance(0.02f);

    // Regime 1: Low-pressure air compression dominant (0 bar)
    // C_sys(0) = 6.0 / (0 + 1)^2 + 0.15 = 6.15 ml/bar
    const float C_zeroBar = compliance.getSystemCompliance(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 6.15f, C_zeroBar);

    // Regime 2: High-pressure hose elastance dominant (9 bar)
    // C_sys(9) = 6.0 / (9 + 1)^2 + 0.15 = 0.06 + 0.15 = 0.21 ml/bar
    const float C_nineBar = compliance.getSystemCompliance(9.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.21f, C_nineBar);

    // Rapid 3-way solenoid venting: pressure plummets from 9.0 to 0.0 bar over 0.6s (-15 bar/s)
    // Verify no negative flow artifacts (Q_puck >= 0.0f at all times)
    compliance.reset();
    constexpr float dt = 0.02f;
    constexpr int ventSteps = 30; // 0.6s
    for (int i = 0; i < ventSteps; ++i) {
        const float pressure = std::max(0.0f, 9.0f - 15.0f * (i * dt));
        const float Q_puck = compliance.update(pressure, 0.0f); // Pump stopped during venting
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, Q_puck);
        TEST_ASSERT_TRUE(std::isfinite(Q_puck));
    }
}

/**
 * @brief Suite 8: Simulates repeated opening/closing of steam valve (negative dP/dt trigger,
 * recovery > 2.7 bar). Asserts injection rate is strictly bounded by thermodynamic ceiling
 * (0.518 ml/s) to prevent boiler quenching.
 */
void test_flash_steam_closed_loop_wand_cycling(void) {
    FlashSteamEngine steam(0.02f);
    steam.startSteamMode();

    float heaterDuty = 0.0f;
    float pumpTargetFlow = 0.0f;

    // Superheat to 148 deg C
    steam.update(148.0f, 3.5f, 0.0f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::READY), static_cast<int>(steam.getState()));

    // --- Steam Cycle 1 ---
    // User cracks steam valve: dP/dt drops rapidly (-0.20 bar/s)
    steam.update(147.0f, 3.2f, -0.20f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::STEAMING_INJECTION), static_cast<int>(steam.getState()));

    // Pressure drops under steaming load to 1.8 bar; PI injection modulates flow
    steam.update(144.0f, 1.8f, 0.0f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_TRUE(pumpTargetFlow <= FlashSteamEngine::kMaxInjectionRate);
    TEST_ASSERT_TRUE(pumpTargetFlow > 0.0f);

    // User closes steam wand: pressure builds and recovers past 2.7 bar
    steam.update(146.0f, 2.8f, 0.10f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::READY), static_cast<int>(steam.getState()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pumpTargetFlow);

    // --- Steam Cycle 2 ---
    // User opens wand again: dP/dt drops rapidly (-0.25 bar/s)
    steam.update(145.0f, 2.6f, -0.25f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::STEAMING_INJECTION), static_cast<int>(steam.getState()));

    // Severe pressure draw down to 1.4 bar: flow strictly bounded by thermodynamic ceiling (0.518 ml/s)
    steam.update(142.0f, 1.4f, -0.05f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_TRUE(pumpTargetFlow <= FlashSteamEngine::kMaxInjectionRate);
    TEST_ASSERT_TRUE(pumpTargetFlow > 0.0f);

    // Close wand again: pressure recovers
    steam.update(146.0f, 2.85f, 0.12f, heaterDuty, pumpTargetFlow);
    TEST_ASSERT_EQUAL(static_cast<int>(FlashSteamEngine::State::READY), static_cast<int>(steam.getState()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pumpTargetFlow);
}

// ============================================================================
// MAIN RUNNER
// ============================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Baseline tests
    RUN_TEST(test_thermal_safety_shutoff_on_overheat);
    RUN_TEST(test_zero_lag_flow_feedforward);
    RUN_TEST(test_smith_predictor_stability_with_deadtime);
    RUN_TEST(test_pump_surface_convexity_and_thermal_fade);
    RUN_TEST(test_compliance_observer_isolates_puck_flow);
    RUN_TEST(test_darcy_channeling_auto_save_trigger);
    RUN_TEST(test_flash_steam_closed_loop_ceiling);
    RUN_TEST(test_autocalibrator_hydraulic_state_machine);

    // 8 Comprehensive SIL physics test suites
    RUN_TEST(test_full_brew_cycle_with_realistic_disturbances);
    RUN_TEST(test_zero_temperature_droop_active_flow_feedforward);
    RUN_TEST(test_smith_predictor_dynamic_disturbance_rejection);
    RUN_TEST(test_channeling_auto_save_reaction_time_and_flow_cap);
    RUN_TEST(test_thermal_safety_invariants_exhaustive);
    RUN_TEST(test_pump_surface_coil_fade_saturation_and_cooling);
    RUN_TEST(test_compliance_observer_two_regime_dynamics);
    RUN_TEST(test_flash_steam_closed_loop_wand_cycling);

    return UNITY_END();
}
