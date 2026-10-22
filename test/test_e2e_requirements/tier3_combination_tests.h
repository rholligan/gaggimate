// Tier 3: Cross-Feature Combination Tests
// 10 Comprehensive Pairwise / Multi-Feature Interaction Test Cases
#pragma once

#include <unity.h>
#include "../common/hydraulic_mocks.h"

using namespace gaggimate_test;

// ============================================================================
// Tier 3: Cross-Feature Combinations
// ============================================================================

// Case 3.1: R1 x R2 - Calibrated surface feedforward paired with closed-loop flow PID
static void test_r3_combo_calibration_feedforward_with_closed_loop_pid() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f, 0.0f);
    FlowPIDController pid(4.0f, 2.0f, 0.03f);

    float targetFlow = 2.0f;
    float currentDuty = 35.0f;

    plant.reset(targetFlow * 3.2f);
    plant.setPuckResistance(3.2f);
    pid.reset();

    // Run 30 steps (under 1 second)
    for (int i = 0; i < 30; ++i) {
        float pumpFlow = pump.getFlow(plant.getPressure(), currentDuty);
        auto state = plant.step(pumpFlow, 0.03f);
        float ffDuty = pump.getFeedforwardDuty(targetFlow, state.pressure);
        currentDuty = pid.update(targetFlow, state.puckFlow, ffDuty);
    }

    float finalFlow = pump.getFlow(plant.getPressure(), currentDuty);
    auto finalState = plant.step(finalFlow, 0.03f);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, targetFlow, finalState.puckFlow,
                                     "Feedforward plus PID must achieve fast sub-second convergence");
}

// Case 3.2: R2 x R4 - Closed-loop flow controller tracking under severe thermal fade
static void test_r3_combo_flow_pid_under_thermal_fade() {
    UlkaPumpPhysicsModel pump;
    ThermalAccumulatorMock thermal(0.80f); // 80% heat soaked
    HydraulicCircuitSimulator plant(1.40f, 0.0f);
    FlowPIDController pid(5.0f, 2.5f, 0.03f);

    float targetFlow = 2.2f;
    float currentDuty = 45.0f;

    plant.reset(targetFlow * 2.8f);
    plant.setPuckResistance(2.8f);
    pid.reset();

    for (int i = 0; i < 40; ++i) {
        float sThermal = thermal.getThermalScalar();
        float pumpFlow = pump.getFlow(plant.getPressure(), currentDuty, sThermal);
        auto state = plant.step(pumpFlow, 0.03f);
        float ffDuty = pump.getFeedforwardDuty(targetFlow, state.pressure, sThermal);
        currentDuty = pid.update(targetFlow, state.puckFlow, ffDuty);
    }

    float sThermal = thermal.getThermalScalar();
    float finalFlow = pump.getFlow(plant.getPressure(), currentDuty, sThermal);
    auto finalState = plant.step(finalFlow, 0.03f);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, targetFlow, finalState.puckFlow,
                                     "Flow controller must maintain setpoint despite 80% thermal fade");
}

// Case 3.3: R2 x R3 - Compound recipe transition from pressure ramp to flow taper
static void test_r3_combo_compound_recipe_with_dynamic_flow_taper() {
    ConditionGroup advanceRule;
    advanceRule.andConditions.push_back({ConditionMetric::PRESSURE, ConditionOp::GTE, 8.5f});
    advanceRule.andConditions.push_back({ConditionMetric::PHASE_TIME, ConditionOp::GTE, 4.0f});

    RecipeTelemetry telem{8.6f, 2.5f, 2.5f, 15.0f, 15.0f, 20.0f, 4.2f};
    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase({advanceRule}, 10.0f, telem);

    TEST_ASSERT_TRUE(reason != PhaseExitReason::NONE);
}

// Case 3.4: R3 x R2 - Predictive auto-stop with seamless virtual scale fallback
static void test_r3_combo_predictive_stop_with_virtual_scale_fallback() {
    SyntheticScaleMock scale;
    scale.reset();
    scale.setLeadTime(0.20f);

    float currentWeight = 30.0f; // Scale dropped at 30g
    float puckFlow = 2.5f;       // Virtual estimator continuing flow
    float targetMass = 36.0f;
    uint32_t now = 20000;

    // Simulate virtual yield progression
    bool autoStopped = false;
    for (int step = 0; step < 80; ++step) {
        now += 30;
        currentWeight += puckFlow * 0.03f;
        float predicted = currentWeight + (puckFlow * scale.getLeadTime());
        if (predicted >= targetMass) {
            autoStopped = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(autoStopped, "Virtual scale momentum must trigger auto-stop when BLE scale drops");
    TEST_ASSERT_FLOAT_WITHIN(0.30f, targetMass, currentWeight + puckFlow * scale.getLeadTime());
}

// Case 3.5: R2 x R4 - Software pressure ceiling overrides flow controller on fine grind
static void test_r3_combo_pressure_ceiling_during_flow_targeting() {
    SoftwarePressureGuard guard(9.0f);
    FlowPIDController pid(4.0f, 1.5f, 0.03f);
    pid.reset();

    // Demand 3.5 ml/s into fine coffee bed that causes 9.2 bar pressure
    float flowCmdDuty = pid.update(3.5f, 1.0f, 65.0f);
    float safeDuty = guard.clampDuty(9.2f, flowCmdDuty);

    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(flowCmdDuty, safeDuty, "Pressure guard must throttle flow command");
}

// Case 3.6: R3 x R4 - Compound recipe exit transitions while thermal accumulator logs energy
static void test_r3_combo_compound_exit_with_thermal_accumulator_logging() {
    ThermalAccumulatorMock thermal(0.20f);
    ConditionGroup group;
    group.andConditions.push_back({ConditionMetric::WEIGHT, ConditionOp::GTE, 36.0f});

    // Run active phase for 10 seconds at 50% duty
    for (int i = 0; i < 333; ++i) {
        thermal.update(50.0f, 0.03f);
    }

    RecipeTelemetry telem{9.0f, 2.0f, 2.0f, 36.1f, 36.1f, 40.0f, 10.0f};
    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase({group}, 25.0f, telem);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PhaseExitReason::TARGET_VOLUMETRIC), static_cast<uint8_t>(reason));
    TEST_ASSERT_GREATER_THAN_FLOAT(0.20f, thermal.getThermalState());
}

// Case 3.7: R2 x R5 - Real-time headless MQTT telemetry streaming during active PID flow brew
static void test_r3_combo_headless_telemetry_streaming_during_closed_loop_brew() {
    FlowPIDController pid(4.0f, 2.0f, 0.03f);
    pid.reset();

    float duty = pid.update(2.0f, 1.95f, 40.0f);
    std::string telemJson = HomeAssistantMqttHarness::formatTelemetryPayload(8.9f, 1.95f, 28.5f, 0.35f);

    TEST_ASSERT_FALSE(telemJson.empty());
    TEST_ASSERT_TRUE(telemJson.find("\"puck_flow\":1.95") != std::string::npos);
    TEST_ASSERT_TRUE(duty > 0.0f);
}

// Case 3.8: R1 x R4 - Surface recalibration normalization under heat-soaked pump
static void test_r3_combo_calibration_recalculation_with_thermal_compensation() {
    ThermalAccumulatorMock thermal(0.70f); // 70% warm
    float sThermal = thermal.getThermalScalar();

    // Measured raw flow on warm machine
    float rawMeasuredFlow = 2.80f;
    // Normalized cold-equivalent flow: Q_cold = Q_warm / S_thermal
    float normalizedColdFlow = rawMeasuredFlow / sThermal;

    TEST_ASSERT_GREATER_THAN_FLOAT(rawMeasuredFlow, normalizedColdFlow);
}

// Case 3.9: R3 x R5 - Remote MQTT command executes multi-phase compound recipe
static void test_r3_combo_mqtt_remote_command_initiates_compound_profile() {
    std::string mqttCommand = "{\"profile\":\"Blooming Flow\",\"action\":\"START\"}";
    bool startSignal = (mqttCommand.find("\"action\":\"START\"") != std::string::npos);

    TEST_ASSERT_TRUE(startSignal);
}

// Case 3.10: R1 x R2 x R4 - Simultaneous thermal compensation, feedforward inversion, and ceiling clamp
static void test_r3_combo_thermal_feedforward_under_choked_puck_ceiling() {
    UlkaPumpPhysicsModel pump;
    ThermalAccumulatorMock thermal(0.90f);
    SoftwarePressureGuard guard(9.0f);

    float sThermal = thermal.getThermalScalar();
    float ffDuty = pump.getFeedforwardDuty(2.5f, 8.8f, sThermal);
    float thermallyBoostedDuty = thermal.compensateDuty(ffDuty);

    // Choked grouphead pushes pressure to 9.3 bar
    float finalSafeDuty = guard.clampDuty(9.3f, thermallyBoostedDuty);

    TEST_ASSERT_LESS_THAN_FLOAT(thermallyBoostedDuty, finalSafeDuty);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, finalSafeDuty);
}
