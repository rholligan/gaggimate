// End-to-End Test Suite for GaggiMate Requirements R1-R5 (Tiers 1-4)
// Total 65 Comprehensive Opaque-Box Tests
// Compatible with PlatformIO Unity Test Runner: pio test -e native -f test_e2e_requirements

#include <unity.h>

#include "tier1_feature_tests.h"
#include "tier2_boundary_tests.h"
#include "tier3_combination_tests.h"
#include "tier4_scenario_tests.h"

// Unity fixture setup and teardown
void setUp(void) {
    // Each test fixture creates its own isolated state and mocks.
}

void tearDown(void) {
    // Clean state after each test execution.
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // ========================================================================
    // TIER 1: CORE FEATURE COVERAGE (25 Tests)
    // ========================================================================
    // R1: Automated Pump Characteristic Calibration Loop
    RUN_TEST(test_r1_open_flow_sweep_evaluates_qmax_vector);
    RUN_TEST(test_r1_stall_sweep_evaluates_pstall_vector);
    RUN_TEST(test_r1_linear_surface_forward_interpolation);
    RUN_TEST(test_r1_inverted_feedforward_duty_solver);
    RUN_TEST(test_r1_nvs_calibration_serialization_and_recovery);

    // R2: Real-Time Flow Estimation & Closed-Loop Controller
    RUN_TEST(test_r2_compliance_compensation_transient_filtering);
    RUN_TEST(test_r2_flow_pid_setpoint_convergence);
    RUN_TEST(test_r2_anti_windup_under_actuator_saturation);
    RUN_TEST(test_r2_bumpless_transfer_between_pressure_and_flow);
    RUN_TEST(test_r2_virtual_scale_yield_accumulation);

    // R3: Event-Driven Conditional Recipe State Machine & Predictive Stop
    RUN_TEST(test_r3_compound_and_condition_evaluation);
    RUN_TEST(test_r3_compound_or_condition_evaluation);
    RUN_TEST(test_r3_exit_reason_reporting);
    RUN_TEST(test_r3_predictive_gravimetric_stop_firing);
    RUN_TEST(test_r3_synchronized_valve_and_pump_cutoff);

    // R4: Mechanical Protections & Thermal Compensation
    RUN_TEST(test_r4_software_pressure_ceiling_throttling);
    RUN_TEST(test_r4_thermal_accumulator_heating_curve);
    RUN_TEST(test_r4_thermal_accumulator_exponential_cooling);
    RUN_TEST(test_r4_thermal_fade_scaling_degradation);
    RUN_TEST(test_r4_thermal_feedforward_duty_compensation);

    // R5: Headless Operation & Home Assistant Telemetry
    RUN_TEST(test_r5_headless_build_macro_contracts);
    RUN_TEST(test_r5_home_assistant_discovery_payload_schema);
    RUN_TEST(test_r5_realtime_mqtt_telemetry_serialization);
    RUN_TEST(test_r5_mqtt_remote_command_routing);
    RUN_TEST(test_r5_websocket_backpressure_and_queue_isolation);

    // ========================================================================
    // TIER 2: BOUNDARY & CORNER CASES (25 Tests)
    // ========================================================================
    // R1: Calibration Boundaries
    RUN_TEST(test_r2_boundary_r1_scale_disconnect_during_open_flow);
    RUN_TEST(test_r2_boundary_r1_blind_basket_detected_in_open_flow);
    RUN_TEST(test_r2_boundary_r1_dry_tank_zero_mass_accumulation);
    RUN_TEST(test_r2_boundary_r1_pressure_exceeding_opv_during_stall);
    RUN_TEST(test_r2_boundary_r1_corrupt_nvs_crc_fallback_to_factory);

    // R2: Flow Controller Boundaries
    RUN_TEST(test_r2_boundary_r2_severe_pressure_spike_compliance_clamp);
    RUN_TEST(test_r2_boundary_r2_choked_puck_infinite_resistance);
    RUN_TEST(test_r2_boundary_r2_puck_blowout_channeling_zero_resistance);
    RUN_TEST(test_r2_boundary_r2_zero_flow_setpoint_bloom_freeze);
    RUN_TEST(test_r2_boundary_r2_deadband_duty_clamping);

    // R3: Recipe State Machine Boundaries
    RUN_TEST(test_r2_boundary_r3_negative_scale_weight_spike_rejection);
    RUN_TEST(test_r2_boundary_r3_ble_packet_drop_momentum_extrapolation);
    RUN_TEST(test_r2_boundary_r3_post_shot_cup_removal_outlier_rejection);
    RUN_TEST(test_r2_boundary_r3_unreachable_compound_target_safety_timeout);
    RUN_TEST(test_r2_boundary_r3_empty_condition_group_fallback);

    // R4: Protections & Thermal Boundaries
    RUN_TEST(test_r2_boundary_r4_hard_overpressure_emergency_cutoff);
    RUN_TEST(test_r2_boundary_r4_thermal_saturation_clamped_to_unity);
    RUN_TEST(test_r2_boundary_r4_maximum_fade_compensation_limit);
    RUN_TEST(test_r2_boundary_r4_reboot_thermal_state_recovery);
    RUN_TEST(test_r2_boundary_r4_zero_duty_negative_cooling_prevention);

    // R5: Headless & Telemetry Boundaries
    RUN_TEST(test_r2_boundary_r5_mqtt_broker_disconnect_ring_buffer_drop);
    RUN_TEST(test_r2_boundary_r5_nan_inf_sensor_payload_sanitization);
    RUN_TEST(test_r2_boundary_r5_malformed_json_command_rejection);
    RUN_TEST(test_r2_boundary_r5_burst_telemetry_memory_pressure);
    RUN_TEST(test_r2_boundary_r5_ble_scale_drop_virtual_scale_arbitration);

    // ========================================================================
    // TIER 3: CROSS-FEATURE COMBINATIONS (10 Tests)
    // ========================================================================
    RUN_TEST(test_r3_combo_calibration_feedforward_with_closed_loop_pid);
    RUN_TEST(test_r3_combo_flow_pid_under_thermal_fade);
    RUN_TEST(test_r3_combo_compound_recipe_with_dynamic_flow_taper);
    RUN_TEST(test_r3_combo_predictive_stop_with_virtual_scale_fallback);
    RUN_TEST(test_r3_combo_pressure_ceiling_during_flow_targeting);
    RUN_TEST(test_r3_combo_compound_exit_with_thermal_accumulator_logging);
    RUN_TEST(test_r3_combo_headless_telemetry_streaming_during_closed_loop_brew);
    RUN_TEST(test_r3_combo_calibration_recalculation_with_thermal_compensation);
    RUN_TEST(test_r3_combo_mqtt_remote_command_initiates_compound_profile);
    RUN_TEST(test_r3_combo_thermal_feedforward_under_choked_puck_ceiling);

    // ========================================================================
    // TIER 4: REAL-WORLD APPLICATION SCENARIOS (5 Tests)
    // ========================================================================
    RUN_TEST(test_r4_scenario_classic_nine_bar_espresso);
    RUN_TEST(test_r4_scenario_blooming_espresso_profile);
    RUN_TEST(test_r4_scenario_turbo_shot_high_flow_low_pressure);
    RUN_TEST(test_r4_scenario_slayer_style_flow_profile);
    RUN_TEST(test_r4_scenario_lever_declining_pressure_profile);

    return UNITY_END();
}
