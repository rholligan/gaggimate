// test_pump_surface_model.cpp
// Comprehensive host-side unit tests for Milestone 1:
// - Physical 2D Pump Displacement Surface Model (Ulka EX5)
// - Piecewise linear / bilinear nodal vector interpolation
// - Linear backpressure throttling: Q(P, Duty) = Qmax(Duty) * (1 - P / Pstall(Duty))
// - Boundary clamping (pressure exceeding stall, deadband conduction, negative pressure)
// - Inverted monotonic feedforward duty root-finding roundtrip
// - Solenoid thermal fade proportional scaling
// - Automated 2-step calibration state machine wizard & safety aborts
// - PressureController integration and mode fallback
//
// PlatformIO execution: pio test -e native -f test_pump_surface_model

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#define ESP_LOGI(tag, fmt, ...) ((void)0)

#include "PressureController/PumpSurfaceModel.cpp"
#include "SimpleKalmanFilter/SimpleKalmanFilter.cpp"
#include "PressureController/PressureController.cpp"
#include "display/core/CalibrationManager.cpp"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// 1. Factory Defaults and Calibration Validation Tests
// ---------------------------------------------------------------------------

void test_factory_calibration_defaults() {
    PumpCalibrationData cal;
    TEST_ASSERT_TRUE(cal.isValid());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, cal.dutyPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, cal.dutyPoints[1]);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, cal.dutyPoints[2]);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, cal.dutyPoints[3]);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cal.dutyPoints[4]);

    TEST_ASSERT_EQUAL_FLOAT(1.8f, cal.qMaxPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(3.5f, cal.qMaxPoints[1]);
    TEST_ASSERT_EQUAL_FLOAT(5.5f, cal.qMaxPoints[2]);
    TEST_ASSERT_EQUAL_FLOAT(7.2f, cal.qMaxPoints[3]);
    TEST_ASSERT_EQUAL_FLOAT(8.5f, cal.qMaxPoints[4]);

    TEST_ASSERT_EQUAL_FLOAT(4.0f, cal.pStallPoints[0]);
    TEST_ASSERT_EQUAL_FLOAT(6.5f, cal.pStallPoints[1]);
    TEST_ASSERT_EQUAL_FLOAT(9.5f, cal.pStallPoints[2]);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, cal.pStallPoints[3]);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, cal.pStallPoints[4]);

    TEST_ASSERT_EQUAL_FLOAT(1.4f, cal.systemCompliance);
    TEST_ASSERT_EQUAL_FLOAT(0.15f, cal.thermalFadeFactor);
}

void test_calibration_validation_checks() {
    PumpCalibrationData bad = PumpCalibrationData();

    // Monotonicity violation in duty
    bad.dutyPoints[2] = 30.0f; // non-monotonic
    TEST_ASSERT_FALSE(bad.isValid());

    // Non-positive Qmax
    bad = PumpCalibrationData();
    bad.qMaxPoints[0] = 0.0f;
    TEST_ASSERT_FALSE(bad.isValid());

    // Non-positive Pstall
    bad = PumpCalibrationData();
    bad.pStallPoints[1] = -1.0f;
    TEST_ASSERT_FALSE(bad.isValid());

    // Non-monotonic Pstall
    bad = PumpCalibrationData();
    bad.pStallPoints[3] = 7.0f; // drops below 9.5
    TEST_ASSERT_FALSE(bad.isValid());

    // Compliance out of reasonable range
    bad = PumpCalibrationData();
    bad.systemCompliance = 0.05f;
    TEST_ASSERT_FALSE(bad.isValid());

    // Thermal fade factor out of range
    bad = PumpCalibrationData();
    bad.thermalFadeFactor = 0.9f;
    TEST_ASSERT_FALSE(bad.isValid());
}

// ---------------------------------------------------------------------------
// 2. Nodal Points and Piecewise Linear Interpolation Tests
// ---------------------------------------------------------------------------

void test_exact_nodal_points() {
    PumpSurfaceModel model;

    // At duty nodes with zero backpressure, delivered flow equals Qmax exactly
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.8f, model.getMaxFlow(20.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, model.getMaxFlow(40.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.5f, model.getMaxFlow(60.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.2f, model.getMaxFlow(80.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(100.0f));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.8f, model.getFlow(0.0f, 20.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, model.getFlow(0.0f, 40.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.5f, model.getFlow(0.0f, 60.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.2f, model.getFlow(0.0f, 80.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(0.0f, 100.0f));

    // Stall pressures at exact nodes
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, model.getStallPressure(20.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.5f, model.getStallPressure(40.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.5f, model.getStallPressure(60.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.0f, model.getStallPressure(80.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.0f, model.getStallPressure(100.0f));
}

void test_piecewise_linear_interpolation() {
    PumpSurfaceModel model;

    // Duty = 30%: midpoint of [20%, 40%]
    // Expected Qmax = (1.8 + 3.5) / 2 = 2.65 ml/s
    // Expected Pstall = (4.0 + 6.5) / 2 = 5.25 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.65f, model.getMaxFlow(30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.25f, model.getStallPressure(30.0f));

    // Duty = 50%: midpoint of [40%, 60%]
    // Expected Qmax = (3.5 + 5.5) / 2 = 4.50 ml/s
    // Expected Pstall = (6.5 + 9.5) / 2 = 8.00 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.50f, model.getMaxFlow(50.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.00f, model.getStallPressure(50.0f));

    // Duty = 70%: midpoint of [60%, 80%]
    // Expected Qmax = (5.5 + 7.2) / 2 = 6.35 ml/s
    // Expected Pstall = (9.5 + 12.0) / 2 = 10.75 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.35f, model.getMaxFlow(70.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.75f, model.getStallPressure(70.0f));

    // Duty = 90%: midpoint of [80%, 100%]
    // Expected Qmax = (7.2 + 8.5) / 2 = 7.85 ml/s
    // Expected Pstall = (12.0 + 14.0) / 2 = 13.00 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.85f, model.getMaxFlow(90.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.00f, model.getStallPressure(90.0f));

    // Duty = 35% (3/4 of [20, 40]):
    // alpha = 15 / 20 = 0.75
    // Qmax = 1.8 + 0.75 * (3.5 - 1.8) = 1.8 + 1.275 = 3.075 ml/s
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.075f, model.getMaxFlow(35.0f));
}

void test_deadband_and_sub_node_conduction() {
    PumpSurfaceModel model;
    model.setDutyDeadband(15.0f);

    // Conduction angle cutoff at or below deadband -> 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(15.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(10.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(-10.0f));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(15.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(5.0f));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 15.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 10.0f));

    // Sub-node linear transition: between deadband (15%) and first node (20%)
    // At 17.5% (midpoint):
    // alpha = (17.5 - 15) / (20 - 15) = 0.5
    // Qmax = 0.5 * 1.8 = 0.90 ml/s
    // Pstall = 0.5 * 4.0 = 2.00 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.90f, model.getMaxFlow(17.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.00f, model.getStallPressure(17.5f));
}

// ---------------------------------------------------------------------------
// 3. Linear Backpressure Throttling Tests
// ---------------------------------------------------------------------------

void test_linear_backpressure_throttling() {
    PumpSurfaceModel model;

    // At 100% duty: Qmax = 8.5 ml/s, Pstall = 14.0 bar
    // Equation: Q = 8.5 * (1 - P / 14.0)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.500f, model.getFlow(0.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.375f, model.getFlow(3.5f, 100.0f));  // P = 0.25 * Pstall
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.250f, model.getFlow(7.0f, 100.0f));  // P = 0.50 * Pstall
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.125f, model.getFlow(10.5f, 100.0f)); // P = 0.75 * Pstall
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.000f, model.getFlow(14.0f, 100.0f)); // P = 1.00 * Pstall

    // At 60% duty: Qmax = 5.5 ml/s, Pstall = 9.5 bar
    // At P = 4.75 bar (half stall):
    // Q = 5.5 * (1 - 0.5) = 2.75 ml/s
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.750f, model.getFlow(4.75f, 60.0f));

    // At 30% duty (interpolated): Qmax = 2.65 ml/s, Pstall = 5.25 bar
    // At P = 2.625 bar:
    // Q = 2.65 * 0.5 = 1.325 ml/s
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.325f, model.getFlow(2.625f, 30.0f));
}

// ---------------------------------------------------------------------------
// 4. Boundary Clamping and Physical Invariants Tests
// ---------------------------------------------------------------------------

void test_boundary_clamping_and_invariants() {
    PumpSurfaceModel model;

    // Pressure strictly >= Pstall must clamp to 0.0 without negative flow or underflow
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(14.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(14.1f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(20.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(100.0f, 100.0f));

    // At 20% duty (Pstall = 4.0 bar):
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(4.0f, 20.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, 20.0f));

    // Negative gauge pressure must clamp to Qmax (no suction over-speeding)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-1.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-10.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.5f, model.getFlow(-2.0f, 60.0f));

    // Duty exceeding 100% clamps to 100% behavior
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.25f, model.getFlow(7.0f, 120.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(110.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.0f, model.getStallPressure(115.0f));
}

// ---------------------------------------------------------------------------
// 5. Inverted Feedforward Duty Root-Finding Roundtrip Tests
// ---------------------------------------------------------------------------

void test_feedforward_duty_root_finding_roundtrip() {
    PumpSurfaceModel model;

    const float targetFlows[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f};
    const float pressures[] = {0.0f, 1.0f, 2.0f, 4.0f, 6.0f, 8.0f, 9.0f};

    for (float P : pressures) {
        const float maxAchievable = model.getFlow(P, 100.0f);
        for (float Qtarget : targetFlows) {
            if (Qtarget < maxAchievable - 0.05f) {
                const float duty = model.getFeedforwardDuty(Qtarget, P);
                TEST_ASSERT_GREATER_OR_EQUAL(model.getDutyDeadband(), duty);
                TEST_ASSERT_LESS_OR_EQUAL(100.0f, duty);

                // Verification of roundtrip: f(f^-1(Q, P), P) approx Q
                const float recoveredFlow = model.getFlow(P, duty);
                TEST_ASSERT_FLOAT_WITHIN(0.005f, Qtarget, recoveredFlow);
            }
        }
    }

    // Edge cases
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(0.0f, 4.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(-2.0f, 4.0f));

    // Target flow beyond physical capacity at high pressure clamps to 100%
    TEST_ASSERT_EQUAL_FLOAT(100.0f, model.getFeedforwardDuty(10.0f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, model.getFeedforwardDuty(5.0f, 12.0f));
}

// ---------------------------------------------------------------------------
// 6. Solenoid Thermal Fade Proportional Scaling Tests
// ---------------------------------------------------------------------------

void test_thermal_fade_proportional_scaling() {
    PumpSurfaceModel model;

    // Fully heat-soaked pump (fade scalar S = 1 - kappa_fade = 0.85)
    const float thermalScalar = 0.85f;

    // Nominal Qmax at 100% is 8.5 -> degraded to 8.5 * 0.85 = 7.225 ml/s
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.225f, model.getMaxFlow(100.0f, thermalScalar));

    // Nominal Pstall at 100% is 14.0 -> degraded to 14.0 * 0.85 = 11.9 bar
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 11.900f, model.getStallPressure(100.0f, thermalScalar));

    // Throttled flow at half stall (P = 5.95 bar):
    // Q = 7.225 * (1 - 5.95 / 11.90) = 7.225 * 0.5 = 3.6125 ml/s
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.6125f, model.getFlow(5.95f, 100.0f, thermalScalar));

    // Feedforward duty under thermal fade:
    // To achieve 3.0 ml/s at 4.0 bar, a faded pump requires strictly higher duty than cold
    const float dutyCold = model.getFeedforwardDuty(3.0f, 4.0f, 1.0f);
    const float dutyHot = model.getFeedforwardDuty(3.0f, 4.0f, thermalScalar);
    TEST_ASSERT_GREATER_THAN(dutyCold, dutyHot);

    // Feedforward roundtrip preserves accuracy under thermal fade
    const float recoveredHot = model.getFlow(4.0f, dutyHot, thermalScalar);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.0f, recoveredHot);
}

// ---------------------------------------------------------------------------
// 7. CalibrationManager State Machine Wizard Tests
// ---------------------------------------------------------------------------

void test_calibration_manager_happy_path() {
    CalibrationManager mgr;
    TEST_ASSERT_EQUAL(CalibrationManager::State::IDLE, mgr.getState());
    TEST_ASSERT_EQUAL_STRING("IDLE", mgr.getStateString());

    // Start calibration -> advances to SWEEP_OPEN_FLOW at 20% duty
    TEST_ASSERT_TRUE(mgr.startCalibration());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_OPEN_FLOW, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_TRUE(mgr.isValveOpenCommand());

    // Synthetic open flow simulation across 5 points
    // Expected Qmax targets: {2.0, 3.8, 5.6, 7.4, 8.8} ml/s
    const float syntheticQmax[5] = {2.0f, 3.8f, 5.6f, 7.4f, 8.8f};
    uint32_t now = 1000;
    float scaleMass = 50.0f;

    for (size_t pt = 0; pt < 5; ++pt) {
        TEST_ASSERT_EQUAL_UINT(pt, mgr.getCurrentPointIndex());
        // Initial tick initializes step
        mgr.update(now, 0.0f, scaleMass, true);

        // Settle dwell (1000ms)
        now += CalibrationManager::SETTLE_DWELL_MS;
        scaleMass += syntheticQmax[pt] * 1.0f;
        mgr.update(now, 0.0f, scaleMass, true);

        // Step dwell remainder (4000ms)
        const uint32_t dtMs = CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS;
        now += dtMs;
        scaleMass += syntheticQmax[pt] * (static_cast<float>(dtMs) / 1000.0f);
        mgr.update(now, 0.0f, scaleMass, true);
    }

    // After 5th point, should transition to AWAIT_BLIND_BASKET
    TEST_ASSERT_EQUAL(CalibrationManager::State::AWAIT_BLIND_BASKET, mgr.getState());
    TEST_ASSERT_EQUAL_STRING("AWAIT_BLIND_BASKET", mgr.getStateString());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());

    // User inserts blind basket and confirms
    TEST_ASSERT_TRUE(mgr.confirmBlindBasket());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_STALL, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, mgr.getActiveDutyCommand());

    // Synthetic stall pressure simulation across 5 points
    // Expected Pstall targets: {4.2, 6.8, 9.8, 12.2, 13.8} bar (below 12 bar limit, let's keep all <= 11.5)
    const float syntheticPstall[5] = {4.2f, 6.8f, 9.2f, 10.5f, 11.5f};

    for (size_t pt = 0; pt < 5; ++pt) {
        TEST_ASSERT_EQUAL_UINT(pt, mgr.getCurrentPointIndex());
        mgr.update(now, syntheticPstall[pt], scaleMass, true);

        // Advance past settle dwell
        now += CalibrationManager::SETTLE_DWELL_MS;
        mgr.update(now, syntheticPstall[pt], scaleMass, true);

        // Advance through step dwell
        const uint32_t dtMs = CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS;
        now += dtMs;
        mgr.update(now, syntheticPstall[pt], scaleMass, true);
    }

    // Should transition through COMPUTING_FIT to SAVED
    TEST_ASSERT_EQUAL(CalibrationManager::State::SAVED, mgr.getState());
    TEST_ASSERT_EQUAL_STRING("SAVED", mgr.getStateString());
    TEST_ASSERT_TRUE(mgr.isCalibrated());

    // Verify fitted model reproduces measured calibration points
    const auto &cal = mgr.getCalibratedData();
    for (size_t i = 0; i < 5; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(0.05f, syntheticQmax[i], cal.qMaxPoints[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, syntheticPstall[i], cal.pStallPoints[i]);
    }
}

void test_calibration_manager_aborts() {
    CalibrationManager mgr;

    // Abort on scale disconnect during open flow sweep
    TEST_ASSERT_TRUE(mgr.startCalibration());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_OPEN_FLOW, mgr.getState());
    mgr.update(1000, 0.0f, 10.0f, false); // scale disconnected
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_NOT_EQUAL(0, strlen(mgr.getLastError()));

    // Abort on pressure runaway during blind stall sweep
    mgr.resetToFactoryDefaults();
    TEST_ASSERT_TRUE(mgr.startCalibration());
    // Fast-forward through open flow step
    uint32_t now = 1000;
    float mass = 100.0f;
    for (size_t i = 0; i < 5; ++i) {
        mgr.update(now, 0.0f, mass, true);
        now += CalibrationManager::SETTLE_DWELL_MS;
        mass += 3.0f;
        mgr.update(now, 0.0f, mass, true);
        now += (CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS);
        mass += 12.0f;
        mgr.update(now, 0.0f, mass, true);
    }
    TEST_ASSERT_EQUAL(CalibrationManager::State::AWAIT_BLIND_BASKET, mgr.getState());
    TEST_ASSERT_TRUE(mgr.confirmBlindBasket());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_STALL, mgr.getState());

    // Inject pressure exceeding 12 bar safety threshold
    mgr.update(now, 13.5f, mass, true);
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_NOT_EQUAL(0, strlen(mgr.getLastError()));

    // User abort
    mgr.resetToFactoryDefaults();
    TEST_ASSERT_TRUE(mgr.startCalibration());
    mgr.abortCalibration("Manual cancel");
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_STRING("Manual cancel", mgr.getLastError());
}

// ---------------------------------------------------------------------------
// 8. PressureController Surface Model Integration Tests
// ---------------------------------------------------------------------------

void test_pressure_controller_surface_model_integration() {
    float rawPressureSetpoint = 9.0f;
    float rawFlowSetpoint = 2.5f;
    float rawPressure = 4.0f;
    float controllerOutput = 0.0f;
    int valveStatus = 1;

    PressureController pc(0.03f, &rawPressureSetpoint, &rawFlowSetpoint, &rawPressure, &controllerOutput, &valveStatus);

    // Initial state: surface model enabled with factory defaults
    TEST_ASSERT_TRUE(pc.isPumpSurfaceModelEnabled());

    // Update in FLOW mode across 100 ticks (3 seconds) to let cosmetic low-pass filter settle
    for (int i = 0; i < 100; ++i) {
        pc.update(PressureController::ControlMode::FLOW);
    }

    // Feedforward duty to achieve 2.5 ml/s at 4.0 bar should be in plausible range (~35% - 65%)
    TEST_ASSERT_GREATER_THAN(20.0f, controllerOutput);
    TEST_ASSERT_LESS_THAN(80.0f, controllerOutput);

    // Verify calculated steady-state pump flow rate corresponds to calibrated surface model
    const float expectedFlow = pc.getPumpSurfaceModel().getFlow(rawPressure, controllerOutput);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expectedFlow, pc.getPumpFlowRate());

    // Setting legacy polynomial coefficients switches mode back to legacy polynomial
    pc.setPumpFlowPolyCoeffs(0.0f, 0.0f, -0.5854f, 10.79f);
    TEST_ASSERT_FALSE(pc.isPumpSurfaceModelEnabled());

    // Setting calibrated data re-engages surface model
    PumpCalibrationData cal;
    pc.setPumpCalibration(cal);
    TEST_ASSERT_TRUE(pc.isPumpSurfaceModelEnabled());
}

// ---------------------------------------------------------------------------
// 9. Numerical Safety, Strict Monotonicity & Rollover Tests (Iteration 2)
// ---------------------------------------------------------------------------

void test_nan_pressure_returns_zero_feedforward_duty() {
    PumpSurfaceModel model;
    const float nanVal = std::numeric_limits<float>::quiet_NaN();
    const float infVal = std::numeric_limits<float>::infinity();

    // 1. NaN in pressure must fail-safe to 0.0% duty (NO 100% runaway)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, nanVal, 1.0f));

    // 2. NaN in target flow must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(nanVal, 5.0f, 1.0f));

    // 3. NaN in thermal scalar must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 5.0f, nanVal));

    // 4. Zero thermal scalar (thermal shutdown) must return 0.0% duty (NO 100% runaway)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 5.0f, 0.0f));

    // 5. Negative thermal scalar must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 5.0f, -0.5f));

    // 6. Infinite target flow must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(infVal, 5.0f, 1.0f));

    // 7. Infinite pressure must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, infVal, 1.0f));

    // 8. Infinite thermal scalar must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 5.0f, infVal));

    // 9. Negative infinity pressure must clamp safely to 0 bar
    const float dutyAtZero = model.getFeedforwardDuty(2.5f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, dutyAtZero, model.getFeedforwardDuty(2.5f, -infVal, 1.0f));

    // 10. Negative target flow must return 0.0% duty
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(-infVal, 5.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(-1.0f, 5.0f, 1.0f));
}

void test_nan_duty_returns_zero_nodal_interpolation() {
    PumpSurfaceModel model;
    const float nanVal = std::numeric_limits<float>::quiet_NaN();

    // 1. NaN duty in interpolateNodalVector via getMaxFlow / getStallPressure
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(nanVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(nanVal));

    // 2. NaN thermal scalar
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(100.0f, nanVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(100.0f, nanVal));

    // 3. NaN parameters in getFlow
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(nanVal, 100.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, nanVal, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, 100.0f, nanVal));

    // 4. Zero and negative thermal scalar in getFlow
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, 100.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, 100.0f, -0.2f));
}

void test_calibration_step_progress_millis_rollover() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    // Initialize step at 16 ms before 32-bit rollover (0xFFFFFFF0)
    const uint32_t stepStart = 0xFFFFFFF0U;
    mgr.update(stepStart, 0.0f, 10.0f, true);

    // Initial tick at stepStart has 0 elapsed ms -> point 0, progress 0%
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, mgr.getStepProgressPercent(stepStart));

    // Progress across rollover: nowMs = 0x00000054 (100 ms elapsed)
    // Elapsed = 100 ms, STEP_DWELL_MS = 5000 ms -> step fraction = 100 / 5000 = 0.02
    // Overall fraction for point 0 = 0.02 / 5 = 0.004 -> 0.4%
    const uint32_t nowAfterRollover100ms = 0x00000054U;
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.4f, mgr.getStepProgressPercent(nowAfterRollover100ms));

    // Progress at midpoint of step: 2500 ms elapsed
    const uint32_t nowAfterRollover2500ms = stepStart + 2500U;
    // Step fraction = 0.5, overall fraction = 0.5 / 5 = 0.1 -> 10.0%
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 10.0f, mgr.getStepProgressPercent(nowAfterRollover2500ms));

    // Progress at end of step: 5000 ms elapsed
    const uint32_t nowAfterRollover5000ms = stepStart + 5000U;
    // Step fraction = 1.0, overall fraction = 1.0 / 5 = 0.2 -> 20.0%
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 20.0f, mgr.getStepProgressPercent(nowAfterRollover5000ms));
}

void test_calibration_data_is_valid_strict_monotonicity() {
    PumpCalibrationData cal;

    // 1. Completely flat stall pressure vector (forgotten blind basket) must be rejected
    cal = PumpCalibrationData();
    for (size_t i = 0; i < PumpCalibrationData::NUM_POINTS; ++i) {
        cal.pStallPoints[i] = 0.2f;
    }
    TEST_ASSERT_FALSE(cal.isValid());

    // 2. Insufficient maximum stall pressure (< 6.0 bar at 100% duty) must be rejected
    cal = PumpCalibrationData();
    cal.pStallPoints[0] = 1.0f;
    cal.pStallPoints[1] = 2.0f;
    cal.pStallPoints[2] = 3.0f;
    cal.pStallPoints[3] = 4.0f;
    cal.pStallPoints[4] = 5.8f;
    TEST_ASSERT_FALSE(cal.isValid());

    // 3. Non-strictly increasing stall pressure (equal adjacent nodes) must be rejected
    cal = PumpCalibrationData();
    cal.pStallPoints[2] = cal.pStallPoints[1]; // 6.5 == 6.5
    TEST_ASSERT_FALSE(cal.isValid());

    // 4. Non-strictly increasing open flow (equal adjacent nodes) must be rejected
    cal = PumpCalibrationData();
    cal.qMaxPoints[3] = cal.qMaxPoints[2]; // 5.5 == 5.5
    TEST_ASSERT_FALSE(cal.isValid());

    // 5. Insufficient maximum open flow (< 3.0 ml/s at 100% duty) must be rejected
    cal = PumpCalibrationData();
    cal.qMaxPoints[4] = 2.8f;
    TEST_ASSERT_FALSE(cal.isValid());

    // 6. Valid calibration profile passes validation
    cal = PumpCalibrationData();
    TEST_ASSERT_TRUE(cal.isValid());
}

// ---------------------------------------------------------------------------
// Unity Test Runner
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_factory_calibration_defaults);
    RUN_TEST(test_calibration_validation_checks);
    RUN_TEST(test_exact_nodal_points);
    RUN_TEST(test_piecewise_linear_interpolation);
    RUN_TEST(test_deadband_and_sub_node_conduction);
    RUN_TEST(test_linear_backpressure_throttling);
    RUN_TEST(test_boundary_clamping_and_invariants);
    RUN_TEST(test_feedforward_duty_root_finding_roundtrip);
    RUN_TEST(test_thermal_fade_proportional_scaling);
    RUN_TEST(test_calibration_manager_happy_path);
    RUN_TEST(test_calibration_manager_aborts);
    RUN_TEST(test_pressure_controller_surface_model_integration);
    RUN_TEST(test_nan_pressure_returns_zero_feedforward_duty);
    RUN_TEST(test_nan_duty_returns_zero_nodal_interpolation);
    RUN_TEST(test_calibration_step_progress_millis_rollover);
    RUN_TEST(test_calibration_data_is_valid_strict_monotonicity);

    return UNITY_END();
}
