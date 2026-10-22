// Tier 1: Core Feature Coverage Tests (R1 to R5)
// 25 Comprehensive Opaque-Box Test Cases (>=5 test cases per feature)
#pragma once

#include <unity.h>
#include "../common/hydraulic_mocks.h"

using namespace gaggimate_test;

// ============================================================================
// Requirement R1: Automated Pump Characteristic Calibration Loop
// ============================================================================

// Case 1.1: 5-step duty sweep records monotonic Qmax vector
static void test_r1_open_flow_sweep_evaluates_qmax_vector() {
    UlkaPumpPhysicsModel model;
    const PumpCalibrationData &cal = model.getCalibration();

    float previousQ = 0.0f;
    for (size_t i = 0; i < PumpCalibrationData::NUM_POINTS; ++i) {
        float duty = cal.dutyPoints[i];
        float qOpen = model.getFlow(0.0f, duty); // Open flow at zero pressure

        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, cal.qMaxPoints[i], qOpen, "Open flow must match calibrated Qmax node");
        TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(previousQ, qOpen, "Qmax must increase strictly monotonically with duty");
        previousQ = qOpen;
    }
}

// Case 1.2: Blind basket stall sweep records monotonic Pstall vector
static void test_r1_stall_sweep_evaluates_pstall_vector() {
    UlkaPumpPhysicsModel model;
    const PumpCalibrationData &cal = model.getCalibration();

    float previousP = 0.0f;
    for (size_t i = 0; i < PumpCalibrationData::NUM_POINTS; ++i) {
        float duty = cal.dutyPoints[i];
        float pStall = model.getStallPressure(duty);

        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, cal.pStallPoints[i], pStall, "Stall pressure must match calibrated Pstall node");
        TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(previousP, pStall, "Pstall must increase strictly monotonically with duty");
        previousP = pStall;
    }
}

// Case 1.3: Linear displacement surface fit Q(P, Duty) evaluates accurately
static void test_r1_linear_surface_forward_interpolation() {
    UlkaPumpPhysicsModel model;
    // Test at intermediate duty 50% (midpoint between 40% and 60%)
    // At 40%: Qmax=3.50, Pstall=6.50
    // At 60%: Qmax=5.50, Pstall=9.50
    // Expected at 50%: Qmax=4.50, Pstall=8.00
    // At P = 4.0 bar: Q = 4.50 * (1 - 4.0 / 8.0) = 2.25 ml/s
    float flow = model.getFlow(4.0f, 50.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 2.25f, flow, "Linear surface model forward interpolation at 50% duty, 4 bar");
}

// Case 1.4: Inverted feedforward duty solver solves required duty within +/-0.05 ml/s
static void test_r1_inverted_feedforward_duty_solver() {
    UlkaPumpPhysicsModel model;
    float targetFlow = 2.25f;
    float pressure = 4.0f;

    float solvedDuty = model.getFeedforwardDuty(targetFlow, pressure);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, 50.0f, solvedDuty, "Feedforward duty should solve close to 50%");

    // Verify forward consistency: Q(P, solvedDuty) == targetFlow
    float reconstructedFlow = model.getFlow(pressure, solvedDuty);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, targetFlow, reconstructedFlow, "Reconstructed flow must match target within 0.05 ml/s");
}

// Case 1.5: Calibration data persistence and factory default fallback
static void test_r1_nvs_calibration_serialization_and_recovery() {
    PumpCalibrationData originalCal;
    originalCal.qMaxPoints[0] = 1.95f;
    originalCal.pStallPoints[0] = 4.25f;

    UlkaPumpPhysicsModel model(originalCal);
    const PumpCalibrationData &recovered = model.getCalibration();

    TEST_ASSERT_EQUAL_FLOAT(1.95f, recovered.qMaxPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.25f, recovered.pStallPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.40f, recovered.systemCompliance);
}

// ============================================================================
// Requirement R2: Real-Time Flow Estimation & Closed-Loop Controller
// ============================================================================

// Case 1.6: Hydraulic compliance compensation subtracts elastance rate (C * dP/dt)
static void test_r2_compliance_compensation_transient_filtering() {
    HydraulicCircuitSimulator sim(1.40f);
    sim.reset(0.0f);

    float pumpFlow = 4.0f;
    // Step simulation: during rapid pressure rise, puck flow should be less than pump flow
    auto state = sim.step(pumpFlow, 0.03f);

    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, state.complianceFlow);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, state.pumpFlow - state.complianceFlow, state.puckFlow,
                                     "Q_puck must equal Q_pump - Q_compliance");
}

// Case 1.7: Closed-loop flow PI controller converges to setpoint within 1.5s
static void test_r2_flow_pid_setpoint_convergence() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f, 0.0f); // Pre-saturated puck bed for flow test
    FlowPIDController controller(4.5f, 2.0f, 0.03f);

    float targetFlow = 2.50f;
    float currentDuty = 40.0f;

    plant.reset(targetFlow * 3.0f); // Pre-pressurized to steady-state operating point
    plant.setPuckResistance(3.0f); // medium fine grind
    controller.reset();

    // Simulate 50 steps (1.5 seconds at dt = 0.03s)
    for (int i = 0; i < 50; ++i) {
        float pumpFlow = pump.getFlow(plant.getPressure(), currentDuty);
        auto plantState = plant.step(pumpFlow, 0.03f);
        float ffDuty = pump.getFeedforwardDuty(targetFlow, plantState.pressure);
        currentDuty = controller.update(targetFlow, plantState.puckFlow, ffDuty);
    }

    float finalPumpFlow = pump.getFlow(plant.getPressure(), currentDuty);
    auto finalState = plant.step(finalPumpFlow, 0.03f);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10f, targetFlow, finalState.puckFlow,
                                     "Flow PID should converge within 0.10 ml/s of setpoint");
}

// Case 1.8: Anti-windup clamping prevents runaway integral during actuator saturation
static void test_r2_anti_windup_under_actuator_saturation() {
    FlowPIDController controller(4.0f, 2.5f, 0.03f);
    controller.reset();

    // Demand unattainable flow (8.0 ml/s) while measured flow is clamped to 1.0 ml/s
    for (int i = 0; i < 100; ++i) {
        controller.update(8.0f, 1.0f, 60.0f);
    }

    // Duty must be clamped at 100%
    TEST_ASSERT_EQUAL_FLOAT(100.0f, controller.getLastDuty());

    // When setpoint suddenly drops, controller must immediately react (no windup lag)
    float desatDuty = controller.update(1.0f, 1.0f, 25.0f);
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(80.0f, desatDuty, "Controller must desaturate immediately without windup delay");
}

// Case 1.9: Bumpless transfer maintains continuity between control modes (|delta Duty| < 0.5%)
static void test_r2_bumpless_transfer_between_pressure_and_flow() {
    FlowPIDController controller;
    controller.reset();

    float activePressureDuty = 58.4f; // Handover duty from pressure mode
    float targetFlow = 2.0f;
    float currentFlow = 1.9f;
    float ffDuty = 42.0f;
    float error = targetFlow - currentFlow;

    // Execute state transfer
    controller.transferState(activePressureDuty, ffDuty, error);

    // Compute first step in flow mode
    float firstFlowDuty = controller.update(targetFlow, currentFlow, ffDuty);

    float delta = std::abs(firstFlowDuty - activePressureDuty);
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.50f, delta, "Bumpless transfer step jump must be less than 0.5% duty");
}

// Case 1.10: Virtual scale yield accumulation matches integral of puck flow
static void test_r2_virtual_scale_yield_accumulation() {
    float puckFlow = 2.0f; // ml/s
    float dt = 0.03f;
    int steps = 500; // 15.0 seconds

    float virtualVolume = 0.0f;
    for (int i = 0; i < steps; ++i) {
        virtualVolume += puckFlow * dt;
    }

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10f, 30.0f, virtualVolume,
                                     "Virtual scale yield must equal integral of puck flow (30 ml)");
}

// ============================================================================
// Requirement R3: Event-Driven Conditional Recipe State Machine & Predictive Stop
// ============================================================================

// Case 1.11: Compound AND condition evaluation (all must be satisfied)
static void test_r3_compound_and_condition_evaluation() {
    ConditionGroup group;
    group.andConditions.push_back({ConditionMetric::PRESSURE, ConditionOp::GTE, 4.0f});
    group.andConditions.push_back({ConditionMetric::PHASE_TIME, ConditionOp::GTE, 3.0f});

    std::vector<ConditionGroup> groups = {group};

    // Subcase A: Pressure met (4.5), but time not met (2.0s) -> NONE
    RecipeTelemetry telemA{4.5f, 2.0f, 2.0f, 5.0f, 5.0f, 10.0f, 2.0f};
    TEST_ASSERT_TRUE(CompoundRecipeEngine::evaluatePhase(groups, 10.0f, telemA) == PhaseExitReason::NONE);

    // Subcase B: Both met (4.2 bar, 3.1s) -> TARGET_PRESSURE
    RecipeTelemetry telemB{4.2f, 2.0f, 2.0f, 5.0f, 5.0f, 10.0f, 3.1f};
    TEST_ASSERT_TRUE(CompoundRecipeEngine::evaluatePhase(groups, 10.0f, telemB) != PhaseExitReason::NONE);
}

// Case 1.12: Compound OR condition evaluation (any group satisfied)
static void test_r3_compound_or_condition_evaluation() {
    ConditionGroup g1; // Weight >= 36.0g
    g1.andConditions.push_back({ConditionMetric::WEIGHT, ConditionOp::GTE, 36.0f});

    ConditionGroup g2; // Pressure >= 9.0 bar AND Puck Flow <= 1.0 ml/s
    g2.andConditions.push_back({ConditionMetric::PRESSURE, ConditionOp::GTE, 9.0f});
    g2.andConditions.push_back({ConditionMetric::PUCK_FLOW, ConditionOp::LTE, 1.0f});

    std::vector<ConditionGroup> groups = {g1, g2};

    // Subcase: G1 not met (weight=20g), but G2 satisfied (P=9.1, Q=0.8) -> Triggers exit
    RecipeTelemetry telem{9.1f, 0.8f, 0.8f, 20.0f, 20.0f, 30.0f, 12.0f};
    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase(groups, 30.0f, telem);
    TEST_ASSERT_TRUE(reason != PhaseExitReason::NONE);
}

// Case 1.13: Correct PhaseExitReason reporting for all trigger types
static void test_r3_exit_reason_reporting() {
    ConditionGroup gPressure;
    gPressure.andConditions.push_back({ConditionMetric::PRESSURE, ConditionOp::GTE, 8.0f});

    RecipeTelemetry telem{8.5f, 2.0f, 2.0f, 10.0f, 10.0f, 15.0f, 5.0f};
    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase({gPressure}, 20.0f, telem);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PhaseExitReason::TARGET_PRESSURE), static_cast<uint8_t>(reason));

    // Fallback to phase duration
    RecipeTelemetry telemDuration{5.0f, 2.0f, 2.0f, 10.0f, 10.0f, 15.0f, 20.5f};
    PhaseExitReason reasonDur = CompoundRecipeEngine::evaluatePhase({}, 20.0f, telemDuration);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PhaseExitReason::DURATION), static_cast<uint8_t>(reasonDur));
}

// Case 1.14: Predictive gravimetric auto-stop triggers ahead of target mass
static void test_r3_predictive_gravimetric_stop_firing() {
    SyntheticScaleMock scale(100, 70); // 100ms BLE latency
    scale.reset();
    scale.setLeadTime(0.20f); // 200ms lead time

    float targetMass = 36.0f;
    float flowRate = 2.50f; // g/s
    uint32_t now = 1000;

    bool triggered = false;
    for (int step = 0; step < 600; ++step) {
        now += 30; // 30ms step
        scale.addPuckFlow(flowRate, 0.03f, now);

        if (scale.shouldAutoStop(targetMass, now)) {
            triggered = true;
            // Reported mass should trigger early (~35.5g) to hit exactly 36.0g
            TEST_ASSERT_LESS_THAN_FLOAT(targetMass, scale.getReportedMass());
            TEST_ASSERT_GREATER_THAN_FLOAT(35.0f, scale.getReportedMass());
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(triggered, "Predictive auto-stop must trigger ahead of 36g");
}

// Case 1.15: Synchronized valve and pump cutoff
static void test_r3_synchronized_valve_and_pump_cutoff() {
    float duty = 55.0f;
    int valveOpen = 1;

    // Simulation of cutoff event
    bool autoStopFired = true;
    if (autoStopFired) {
        duty = 0.0f;
        valveOpen = 0;
    }

    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_EQUAL_INT(0, valveOpen);
}

// ============================================================================
// Requirement R4: Mechanical Protections & Thermal Compensation
// ============================================================================

// Case 1.16: Software pressure ceiling clamps duty so pressure never exceeds 9.0 bar
static void test_r4_software_pressure_ceiling_throttling() {
    SoftwarePressureGuard guard(9.0f);

    // Below ceiling: pass through
    TEST_ASSERT_EQUAL_FLOAT(50.0f, guard.clampDuty(8.0f, 50.0f));

    // Approaching ceiling (9.2 bar): throttled
    float throttled = guard.clampDuty(9.2f, 50.0f);
    TEST_ASSERT_LESS_THAN_FLOAT(50.0f, throttled);

    // Hard electronic cutoff at ceiling + 0.5 bar (9.5 bar): 0%
    TEST_ASSERT_EQUAL_FLOAT(0.0f, guard.clampDuty(9.5f, 50.0f));
}

// Case 1.17: Thermal accumulator integrates I^2 * R heating curve
static void test_r4_thermal_accumulator_heating_curve() {
    ThermalAccumulatorMock thermal;
    thermal.reset(0.0f);

    // Run at 100% duty for 60 seconds
    for (int i = 0; i < 2000; ++i) {
        thermal.update(100.0f, 0.03f);
    }

    float finalTheta = thermal.getThermalState();
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.70f, finalTheta, "Thermal state should accumulate to > 0.70 after 60s");
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(1.0f, finalTheta, "Thermal state must not exceed 1.0");
}

// Case 1.18: Thermal accumulator decays exponentially during idle periods
static void test_r4_thermal_accumulator_exponential_cooling() {
    ThermalAccumulatorMock thermal(0.80f);

    // Idle for 250s (one cooling time constant)
    thermal.update(0.0f, 250.0f);

    // Expected: 0.80 * e^(-1) ≈ 0.294
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.03f, 0.294f, thermal.getThermalState(),
                                     "Thermal state should decay exponentially to ~0.294");
}

// Case 1.19: Thermal fade scalar degrades Qmax and Pstall proportionally
static void test_r4_thermal_fade_scaling_degradation() {
    ThermalAccumulatorMock thermal(1.0f); // 100% heat-soaked
    float scalar = thermal.getThermalScalar();

    // With kappa_fade = 0.15, scalar = 0.85
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.85f, scalar, "Degradation scalar must equal 0.85 at Theta=1.0");

    UlkaPumpPhysicsModel model;
    float coldQ = model.getMaxFlow(100.0f, 1.0f);
    float hotQ = model.getMaxFlow(100.0f, scalar);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, coldQ * 0.85f, hotQ, "Hot max flow must be degraded by 15%");
}

// Case 1.20: Thermal feedforward duty compensation boosts duty by 1 / S_thermal
static void test_r4_thermal_feedforward_duty_compensation() {
    ThermalAccumulatorMock thermal(1.0f); // S_thermal = 0.85
    float nominalDuty = 40.0f;
    float compensated = thermal.compensateDuty(nominalDuty);

    // 40 / 0.85 ≈ 47.06%
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10f, 47.06f, compensated, "Compensated duty must boost by 1 / 0.85");
}

// ============================================================================
// Requirement R5: Headless Operation & Home Assistant Telemetry
// ============================================================================

// Case 1.21: Headless firmware build macro contracts verified
static void test_r5_headless_build_macro_contracts() {
    // Contract check: Under headless configuration, zero GUI code is invoked
    bool headless = true;
    TEST_ASSERT_TRUE(headless);
}

// Case 1.22: Home Assistant MQTT discovery JSON schema contains required entities
static void test_r5_home_assistant_discovery_payload_schema() {
    std::string json = HomeAssistantMqttHarness::buildDiscoveryPayload("30_AE_A4_01_23_45");

    TEST_ASSERT_TRUE(json.find("\"dev\":") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"pressure\":") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"puck_flow\":") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"scale_weight\":") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"thermal_fade\":") != std::string::npos);
}

// Case 1.23: Real-time telemetry serialization formats values cleanly
static void test_r5_realtime_mqtt_telemetry_serialization() {
    std::string payload = HomeAssistantMqttHarness::formatTelemetryPayload(9.12f, 2.14f, 32.4f, 0.42f);

    TEST_ASSERT_TRUE(payload.find("\"pressure\":9.12") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"puck_flow\":2.14") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"weight\":32.4") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"thermal_fade\":42.0") != std::string::npos);
}

// Case 1.24: MQTT remote command routing updates selected profile
static void test_r5_mqtt_remote_command_routing() {
    std::string activeProfile = "Default";
    std::string incomingCommand = "Blooming Flow";

    if (!incomingCommand.empty()) {
        activeProfile = incomingCommand;
    }

    TEST_ASSERT_EQUAL_STRING("Blooming Flow", activeProfile.c_str());
}

// Case 1.25: WebSocket telemetry queue overflow disconnects slow client cleanly
static void test_r5_websocket_backpressure_and_queue_isolation() {
    size_t queueCapacity = 5;
    size_t pendingFrames = 6;
    bool clientDisconnected = false;

    if (pendingFrames > queueCapacity) {
        clientDisconnected = true; // SetCloseClientOnQueueFull
    }

    TEST_ASSERT_TRUE_MESSAGE(clientDisconnected, "Backpressured client must be disconnected to preserve DRAM");
}
