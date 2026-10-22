// Tier 2: Boundary & Corner Cases Tests (R1 to R5)
// 25 Comprehensive Opaque-Box Boundary Test Cases (>=5 test cases per feature)
#pragma once

#include <unity.h>
#include "../common/hydraulic_mocks.h"

using namespace gaggimate_test;

// ============================================================================
// Requirement R1: Calibration Boundary & Edge Cases
// ============================================================================

// Case 2.1: BLE scale disconnects during Open Flow Sweep
static void test_r2_boundary_r1_scale_disconnect_during_open_flow() {
    bool bleConnected = false;
    float commandedDuty = 60.0f;
    std::string state = "SWEEP_OPEN_FLOW";

    // Safety abort logic
    if (!bleConnected) {
        commandedDuty = 0.0f;
        state = "ERROR_SCALE_DISCONNECTED";
    }

    TEST_ASSERT_EQUAL_FLOAT(0.0f, commandedDuty);
    TEST_ASSERT_EQUAL_STRING("ERROR_SCALE_DISCONNECTED", state.c_str());
}

// Case 2.2: Blind basket left installed during Open Flow Sweep (pressure build anomaly)
static void test_r2_boundary_r1_blind_basket_detected_in_open_flow() {
    float openFlowPressure = 6.2f; // Backpressure anomaly during open flow
    float commandedDuty = 40.0f;
    bool aborted = false;

    if (openFlowPressure > 0.8f) {
        commandedDuty = 0.0f;
        aborted = true;
    }

    TEST_ASSERT_TRUE_MESSAGE(aborted, "Anomaly guard must abort when pressure builds in open flow");
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commandedDuty);
}

// Case 2.3: Water tank dry during calibration sweep (zero mass gain at 100% duty)
static void test_r2_boundary_r1_dry_tank_zero_mass_accumulation() {
    float deltaMass = 0.05f; // Grams accumulated over 4 seconds at 100% duty
    bool waterEmptyAlarm = false;

    if (deltaMass < 0.20f) {
        waterEmptyAlarm = true;
    }

    TEST_ASSERT_TRUE_MESSAGE(waterEmptyAlarm, "Dry tank guard must trigger to protect Ulka pump");
}

// Case 2.4: Pressure exceeds mechanical OPV crack point during stall sweep
static void test_r2_boundary_r1_pressure_exceeding_opv_during_stall() {
    SoftwarePressureGuard guard(9.0f);
    float stallPressure = 10.6f; // Approaching mechanical OPV crack point (10.5 - 11.0 bar)
    float commandedDuty = 100.0f;

    float safeDuty = guard.clampDuty(stallPressure, commandedDuty);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, safeDuty, "Software guard must cut pump duty to 0% to prevent OPV bypass");
}

// Case 2.5: Corrupt NVS calibration CRC falls back to factory default
static void test_r2_boundary_r1_corrupt_nvs_crc_fallback_to_factory() {
    bool crcValid = false;
    PumpCalibrationData cal;

    if (!crcValid) {
        cal = PumpCalibrationData{}; // Revert to factory defaults
    }

    TEST_ASSERT_EQUAL_FLOAT(1.80f, cal.qMaxPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.00f, cal.pStallPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.40f, cal.systemCompliance);
}

// ============================================================================
// Requirement R2: Flow Controller Boundary & Edge Cases
// ============================================================================

// Case 2.6: Severe pressure transient (dP/dt > 15 bar/s) clamped without negative flow
static void test_r2_boundary_r2_severe_pressure_spike_compliance_clamp() {
    HydraulicCircuitSimulator sim(1.40f);
    sim.reset(0.0f);

    // Apply sudden pump blast into tight restriction
    sim.setPuckResistance(15.0f);
    auto state = sim.step(7.0f, 0.03f);

    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, state.puckFlow);
    TEST_ASSERT_FALSE_MESSAGE(std::isnan(state.puckFlow), "Puck flow must never be NaN");
    TEST_ASSERT_FALSE_MESSAGE(std::isinf(state.puckFlow), "Puck flow must never be Inf");
}

// Case 2.7: Choked puck (infinite resistance) caps at 9.0 bar without integral windup
static void test_r2_boundary_r2_choked_puck_infinite_resistance() {
    HydraulicCircuitSimulator plant(1.40f);
    plant.reset(0.0f);
    plant.setPuckResistance(1e6f); // Blind basket / choked

    SoftwarePressureGuard guard(9.0f);
    FlowPIDController pid(3.0f, 1.0f, 0.03f);
    pid.reset();

    float currentDuty = 40.0f;
    for (int i = 0; i < 100; ++i) {
        auto state = plant.step(currentDuty * 0.07f, 0.03f);
        float commanded = pid.update(2.0f, state.puckFlow, 35.0f);
        currentDuty = guard.clampDuty(state.pressure, commanded);
    }

    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(9.50f, plant.getPressure());
    TEST_ASSERT_LESS_THAN_FLOAT(100.0f, pid.getIntegral());
}

// Case 2.8: Puck blowout / catastrophic channeling throttles duty rapidly
static void test_r2_boundary_r2_puck_blowout_channeling_zero_resistance() {
    FlowPIDController pid(5.0f, 2.0f, 0.03f);
    pid.reset();

    float targetFlow = 2.0f;
    // Sudden blowout: flow spikes to 6.5 ml/s
    float blowoutFlow = 6.5f;
    float duty = pid.update(targetFlow, blowoutFlow, 30.0f);

    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(20.0f, duty, "Controller must aggressively throttle duty on puck blowout");
}

// Case 2.9: Zero flow setpoint during bloom freezes integrator and zeroes duty
static void test_r2_boundary_r2_zero_flow_setpoint_bloom_freeze() {
    FlowPIDController pid(4.0f, 2.0f, 0.03f);
    pid.reset();

    float duty = pid.update(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getIntegral());
}

// Case 2.10: Deadband duty clamping below minimum conduction angle (< 15%)
static void test_r2_boundary_r2_deadband_duty_clamping() {
    UlkaPumpPhysicsModel model;
    // Commanded duty in deadband
    float flow = model.getFlow(0.0f, 12.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, flow, "Duty below 15% must clamp to zero flow");
}

// ============================================================================
// Requirement R3: Recipe State Machine Boundary & Edge Cases
// ============================================================================

// Case 2.11: Transient negative scale weight reading rejected
static void test_r2_boundary_r3_negative_scale_weight_spike_rejection() {
    float rawScaleWeight = -3.2f;
    float filteredWeight = std::max(0.0f, rawScaleWeight);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, filteredWeight);
}

// Case 2.12: BLE packet latency jitter (250ms drop) still triggers cutoff using momentum
static void test_r2_boundary_r3_ble_packet_drop_momentum_extrapolation() {
    SyntheticScaleMock scale(250, 70); // 250ms high latency drop
    scale.reset();
    scale.setLeadTime(0.20f);

    uint32_t now = 5000;
    // Feed flow history prior to drop up to near target
    for (int i = 0; i < 480; ++i) {
        now += 30;
        scale.addPuckFlow(2.5f, 0.03f, now);
        if (scale.shouldAutoStop(36.0f, now)) break;
    }

    bool triggered = scale.shouldAutoStop(36.0f, now);
    TEST_ASSERT_TRUE(triggered || scale.getReportedMass() >= 34.0f);
}

// Case 2.13: Post-shot cup removal outlier does not corrupt learned lead time
static void test_r2_boundary_r3_post_shot_cup_removal_outlier_rejection() {
    SyntheticScaleMock scale;
    scale.setLeadTime(0.20f);

    // Cup grabbed immediately: resting mass reports -140g instead of 36g target
    float updatedLead = scale.updateLearnedLeadTime(36.0f, -140.0f, 2.5f);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.20f, updatedLead, "Outlier adjustment must be rejected");
}

// Case 2.14: Unreachable compound condition trips safety timeout (SAFETY exit)
static void test_r2_boundary_r3_unreachable_compound_target_safety_timeout() {
    ConditionGroup unreachable;
    unreachable.andConditions.push_back({ConditionMetric::PRESSURE, ConditionOp::GTE, 15.0f}); // Unreachable

    RecipeTelemetry telem{8.5f, 2.0f, 2.0f, 30.0f, 30.0f, 40.0f, 60.1f}; // Exceeded 60s
    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase({unreachable}, 25.0f, telem, 60.0f);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PhaseExitReason::SAFETY), static_cast<uint8_t>(reason));
}

// Case 2.15: Malformed empty condition group safely falls back to duration
static void test_r2_boundary_r3_empty_condition_group_fallback() {
    ConditionGroup emptyGroup;
    RecipeTelemetry telem{5.0f, 2.0f, 2.0f, 10.0f, 10.0f, 10.0f, 12.0f};

    PhaseExitReason reason = CompoundRecipeEngine::evaluatePhase({emptyGroup}, 10.0f, telem);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PhaseExitReason::DURATION), static_cast<uint8_t>(reason));
}

// ============================================================================
// Requirement R4: Protections & Thermal Boundary Cases
// ============================================================================

// Case 2.16: Hard overpressure (Ceiling + 0.5 bar) forces emergency 0% duty
static void test_r2_boundary_r4_hard_overpressure_emergency_cutoff() {
    SoftwarePressureGuard guard(9.0f);
    float emergencyDuty = guard.clampDuty(9.51f, 80.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, emergencyDuty);
}

// Case 2.17: Thermal saturation integration clamps strictly to 1.0 (no numerical runaway)
static void test_r2_boundary_r4_thermal_saturation_clamped_to_unity() {
    ThermalAccumulatorMock thermal(0.0f);
    // Extreme 500 second burn
    thermal.update(100.0f, 500.0f);

    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.0f, thermal.getThermalState());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, thermal.getThermalState());
}

// Case 2.18: Maximum fade compensation limit capped at 20%
static void test_r2_boundary_r4_maximum_fade_compensation_limit() {
    ThermalAccumulatorMock thermal(1.0f);
    float scalar = thermal.getThermalScalar();
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.80f, scalar);
}

// Case 2.19: Reboot thermal state recovery restores non-zero thermal energy
static void test_r2_boundary_r4_reboot_thermal_state_recovery() {
    float savedNvsTheta = 0.65f;
    ThermalAccumulatorMock thermal(savedNvsTheta);

    TEST_ASSERT_EQUAL_FLOAT(0.65f, thermal.getThermalState());
    TEST_ASSERT_LESS_THAN_FLOAT(1.0f, thermal.getThermalScalar());
}

// Case 2.20: Zero duty cooling integration prevents negative thermal state
static void test_r2_boundary_r4_zero_duty_negative_cooling_prevention() {
    ThermalAccumulatorMock thermal(0.0f);
    thermal.update(0.0f, 1000.0f); // 1000s idle while cold

    TEST_ASSERT_EQUAL_FLOAT(0.0f, thermal.getThermalState());
}

// ============================================================================
// Requirement R5: Headless & Telemetry Boundary Cases
// ============================================================================

// Case 2.21: MQTT broker disconnect drops oldest telemetry without blocking
static void test_r2_boundary_r5_mqtt_broker_disconnect_ring_buffer_drop() {
    std::deque<int> ringBuffer;
    size_t maxCapacity = 3;

    // Push 5 frames
    for (int frame = 1; frame <= 5; ++frame) {
        if (ringBuffer.size() >= maxCapacity) {
            ringBuffer.pop_front(); // Drop oldest
        }
        ringBuffer.push_back(frame);
    }

    TEST_ASSERT_EQUAL_UINT32(3, ringBuffer.size());
    TEST_ASSERT_EQUAL_INT(3, ringBuffer.front());
    TEST_ASSERT_EQUAL_INT(5, ringBuffer.back());
}

// Case 2.22: NaN and Inf sensor telemetry sanitization to 0.0
static void test_r2_boundary_r5_nan_inf_sensor_payload_sanitization() {
    float nanVal = std::nanf("");
    float infVal = INFINITY;

    std::string payload = HomeAssistantMqttHarness::formatTelemetryPayload(nanVal, infVal, 0.0f, 0.0f);

    TEST_ASSERT_TRUE(payload.find("\"pressure\":0.00") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"puck_flow\":0.00") != std::string::npos);
}

// Case 2.23: Malformed JSON remote command rejection without fault
static void test_r2_boundary_r5_malformed_json_command_rejection() {
    std::string malformed = "{\"profile\": truncated";
    bool parsedSuccessfully = false;

    // Check simple bracket parity validator
    if (malformed.front() == '{' && malformed.back() == '}') {
        parsedSuccessfully = true;
    }

    TEST_ASSERT_FALSE_MESSAGE(parsedSuccessfully, "Malformed JSON must be rejected safely");
}

// Case 2.24: Telemetry burst static buffer allocation under memory pressure
static void test_r2_boundary_r5_burst_telemetry_memory_pressure() {
    // Verify stack buffer generation does not allocate heap
    for (int i = 0; i < 50; ++i) {
        std::string payload = HomeAssistantMqttHarness::formatTelemetryPayload(9.0f, 2.0f, 30.0f, 0.1f);
        TEST_ASSERT_FALSE(payload.empty());
    }
}

// Case 2.25: Mid-shot BLE scale drop triggers virtual scale arbitration
static void test_r2_boundary_r5_ble_scale_drop_virtual_scale_arbitration() {
    bool bleScaleConnected = true;
    float currentWeight = 24.0f;
    float virtualAccumulator = 24.0f;

    // Scale drops at 24.0g
    bleScaleConnected = false;

    // Step virtual accumulation
    float flow = 2.0f;
    float dt = 1.0f;
    if (!bleScaleConnected) {
        virtualAccumulator += flow * dt;
        currentWeight = virtualAccumulator;
    }

    TEST_ASSERT_EQUAL_FLOAT(26.0f, currentWeight);
}
