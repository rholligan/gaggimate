// test_adversarial_m1.cpp
// Empirical Adversarial Stress Test Harness for GaggiMate Milestone 1:
// - Extreme boundary violations and non-linear inputs on PumpSurfaceModel
// - Asymptotic pressure behavior (P = 20 bar, 100 bar, +inf, -5 bar, -inf, NaN)
// - Out-of-range duty cycles (-10%, 0%, 5%, 15%, 105%, 200%, NaN)
// - Monotonicity verification across fine duty & pressure grids
// - Inverted feedforward root-finding convergence and edge cases
// - Thermal fade saturation & boundary clamps
// - CalibrationManager stress: rapid aborts, mid-sweep scale disconnects, runaway pressures, dry tank detection, non-monotonic fits, phase timeouts
//
// PlatformIO: pio test -e native -f "*adversarial_m1*"

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#define ESP_LOGI(tag, fmt, ...) ((void)0)

#include "PressureController/PumpSurfaceModel.cpp"
#include "SimpleKalmanFilter/SimpleKalmanFilter.cpp"
#include "PressureController/PressureController.cpp"
#include "display/core/CalibrationManager.cpp"

void setUp() {}
void tearDown() {}

// ===========================================================================
// 1. PumpSurfaceModel Extreme Boundary & Non-Linear Inputs
// ===========================================================================

void test_adversarial_extreme_pressure_clamping() {
    PumpSurfaceModel model;

    // At 100% duty, stall pressure is 14.0 bar, Qmax is 8.5 ml/s
    const float pStall = model.getStallPressure(100.0f);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, pStall);

    // Exact stall boundary
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(pStall, 100.0f));

    // Barely above stall
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(14.001f, 100.0f));

    // Well above stall (20 bar, 100 bar, 1e6 bar)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(20.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(100.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(1e6f, 100.0f));

    // Positive infinity pressure
    const float infPressure = std::numeric_limits<float>::infinity();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(infPressure, 100.0f));

    // Negative pressures must clamp safely to Qmax (8.5 ml/s) without cavitation/suction artifacts
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-0.001f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-1.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-5.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(-100.0f, 100.0f));

    // Negative infinity pressure
    const float negInfPressure = -std::numeric_limits<float>::infinity();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getFlow(negInfPressure, 100.0f));

    // NaN pressure input must not crash, throw, or return NaN
    const float nanVal = std::numeric_limits<float>::quiet_NaN();
    float flowNan = model.getFlow(nanVal, 100.0f);
    TEST_ASSERT_FALSE(std::isnan(flowNan));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, flowNan);
}

void test_adversarial_extreme_duty_cycles() {
    PumpSurfaceModel model;

    // Below or at deadband (15%) -> must yield strictly 0.0 ml/s flow and 0 stall pressure
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, -10.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 5.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 14.999f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 15.0f));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(5.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(15.0f));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(5.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(15.0f));

    // Barely above deadband (15.01%) -> smooth non-negative flow
    float flowJustAbove = model.getFlow(0.0f, 15.01f);
    TEST_ASSERT_TRUE(flowJustAbove >= 0.0f);
    TEST_ASSERT_TRUE(flowJustAbove < 0.1f);

    // Extreme duty cycles > 100% must clamp to 100% saturation values (Qmax = 8.5, Pstall = 14.0)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(105.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(150.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(1000.0f));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.0f, model.getStallPressure(105.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.0f, model.getStallPressure(200.0f));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.25f, model.getFlow(7.0f, 105.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.25f, model.getFlow(7.0f, 500.0f));
}

void test_adversarial_monotonicity_fine_grid() {
    PumpSurfaceModel model;

    // 1. Monotonicity across Duty for various pressures:
    // For any fixed pressure, flow must be monotonically non-decreasing with duty
    const float testPressures[] = {0.0f, 1.5f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.5f, 15.0f};

    for (float P : testPressures) {
        float previousFlow = -1.0f;
        for (float duty = 0.0f; duty <= 100.0f; duty += 0.5f) {
            float flow = model.getFlow(P, duty);
            TEST_ASSERT_TRUE_MESSAGE(flow >= 0.0f, "Delivered flow must never be negative");
            if (previousFlow >= 0.0f) {
                TEST_ASSERT_TRUE_MESSAGE(
                    flow >= previousFlow - 1e-5f,
                    "Flow must be monotonically non-decreasing with respect to duty cycle"
                );
            }
            previousFlow = flow;
        }
    }

    // 2. Monotonicity across Pressure for various duties:
    // For any fixed duty, flow must be monotonically non-increasing with pressure
    const float testDuties[] = {15.0f, 17.5f, 20.0f, 35.0f, 50.0f, 65.0f, 80.0f, 95.0f, 100.0f};

    for (float duty : testDuties) {
        float previousFlow = 1e6f;
        for (float P = 0.0f; P <= 16.0f; P += 0.2f) {
            float flow = model.getFlow(P, duty);
            TEST_ASSERT_TRUE_MESSAGE(flow >= 0.0f, "Delivered flow must never be negative");
            TEST_ASSERT_TRUE_MESSAGE(
                flow <= previousFlow + 1e-5f,
                "Flow must be monotonically non-increasing with respect to backpressure"
            );
            previousFlow = flow;
        }
    }
}

void test_adversarial_feedforward_inversion_boundary_matrix() {
    PumpSurfaceModel model;

    // Achievable combinations: verify roundtrip f(f^-1(Q, P), P) ~= Q
    for (float P = 0.0f; P <= 10.0f; P += 1.0f) {
        const float maxFlowAtP = model.getFlow(P, 100.0f);
        if (maxFlowAtP <= 0.2f) continue;

        for (float qTarget = 0.2f; qTarget < maxFlowAtP - 0.1f; qTarget += 0.5f) {
            float duty = model.getFeedforwardDuty(qTarget, P);
            TEST_ASSERT_TRUE(duty >= model.getDutyDeadband());
            TEST_ASSERT_TRUE(duty <= 100.0f);

            float recovered = model.getFlow(P, duty);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.005f, qTarget, recovered, "Feedforward duty roundtrip must match target within 0.005 ml/s");
        }
    }

    // Target flow <= 0 -> duty must be 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(0.0f, 5.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(-5.0f, 5.0f));

    // Target flow unobtainable even at 100% duty -> duty must saturate to 100%
    TEST_ASSERT_EQUAL_FLOAT(100.0f, model.getFeedforwardDuty(10.0f, 0.0f)); // Qmax at 0 bar is 8.5
    TEST_ASSERT_EQUAL_FLOAT(100.0f, model.getFeedforwardDuty(5.0f, 9.0f));  // Qmax at 9 bar is 3.03
    TEST_ASSERT_EQUAL_FLOAT(100.0f, model.getFeedforwardDuty(1.0f, 20.0f)); // Stalled at 20 bar

    // Negative pressure in feedforward solver must clamp safely to 0 bar
    float dutyNegP = model.getFeedforwardDuty(3.5f, -5.0f);
    float dutyZeroP = model.getFeedforwardDuty(3.5f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, dutyZeroP, dutyNegP);
}

void test_adversarial_thermal_fade_clamping() {
    PumpSurfaceModel model;

    // Normal scalar 1.0f
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(100.0f, 1.0f));

    // Partial fade 0.8f
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.8f, model.getMaxFlow(100.0f, 0.8f));

    // Zero scalar (thermal shutdown / cold start disable)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(100.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(100.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 100.0f, 0.0f));

    // Negative scalar must clamp to 0.0f
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(100.0f, -0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(100.0f, -1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(0.0f, 100.0f, -0.2f));

    // Over-unity scalar (>1.0f) must clamp to 1.0f
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.5f, model.getMaxFlow(100.0f, 1.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.0f, model.getStallPressure(100.0f, 2.0f));
}

// ===========================================================================
// 2. CalibrationManager Adversarial Stress Tests
// ===========================================================================

void test_adversarial_calibration_rapid_aborts() {
    CalibrationManager mgr;

    // Abort from IDLE
    mgr.abortCalibration("Abort while idle");
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());
    TEST_ASSERT_EQUAL_STRING("Abort while idle", mgr.getLastError());

    // 100 rapid repeated aborts must not crash or leak
    for (int i = 0; i < 100; ++i) {
        mgr.abortCalibration("Spam abort");
        TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    }

    // Restart cleanly after spam abort
    TEST_ASSERT_TRUE(mgr.startCalibration());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_OPEN_FLOW, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_TRUE(mgr.isValveOpenCommand());

    // Abort during SWEEP_OPEN_FLOW
    mgr.abortCalibration("Abort in open flow");
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());

    // Restart and advance to AWAIT_BLIND_BASKET
    TEST_ASSERT_TRUE(mgr.startCalibration());
    uint32_t now = 1000;
    float mass = 10.0f;
    for (size_t pt = 0; pt < 5; ++pt) {
        mgr.update(now, 0.0f, mass, true);
        now += CalibrationManager::SETTLE_DWELL_MS;
        mass += 2.0f;
        mgr.update(now, 0.0f, mass, true);
        now += (CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS);
        mass += 10.0f;
        mgr.update(now, 0.0f, mass, true);
    }
    TEST_ASSERT_EQUAL(CalibrationManager::State::AWAIT_BLIND_BASKET, mgr.getState());

    // Abort in AWAIT_BLIND_BASKET
    mgr.abortCalibration("Abort during basket swap");
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());

    // Confirm blind basket while in ERROR_ABORT must be rejected
    TEST_ASSERT_FALSE(mgr.confirmBlindBasket());
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
}

void test_adversarial_calibration_mid_sweep_scale_disconnects() {
    CalibrationManager mgr;

    // 1. Disconnect on the very first update tick of Open Flow
    TEST_ASSERT_TRUE(mgr.startCalibration());
    mgr.update(100, 0.0f, 0.0f, false);
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());

    // 2. Disconnect during settling period of point 2 (60% duty)
    TEST_ASSERT_TRUE(mgr.startCalibration());
    uint32_t now = 1000;
    float mass = 10.0f;
    // Step 0 (20%)
    mgr.update(now, 0.0f, mass, true);
    now += 1000; mass += 2.0f; mgr.update(now, 0.0f, mass, true);
    now += 4000; mass += 8.0f; mgr.update(now, 0.0f, mass, true);
    // Step 1 (40%)
    mgr.update(now, 0.0f, mass, true);
    now += 1000; mass += 3.5f; mgr.update(now, 0.0f, mass, true);
    now += 4000; mass += 14.0f; mgr.update(now, 0.0f, mass, true);

    TEST_ASSERT_EQUAL_UINT(2, mgr.getCurrentPointIndex());
    // Point 2 starts, scale drops after 500ms
    now += 500;
    mgr.update(now, 0.0f, mass, false); // Scale disconnected mid-step!
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());
    TEST_ASSERT_NOT_EQUAL(0, strlen(mgr.getLastError()));
}

void test_adversarial_calibration_runaway_pressure_spike() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    // Advance through Open Flow
    uint32_t now = 1000;
    float mass = 10.0f;
    for (size_t pt = 0; pt < 5; ++pt) {
        mgr.update(now, 0.0f, mass, true);
        now += 1000; mass += 2.0f; mgr.update(now, 0.0f, mass, true);
        now += 4000; mass += 10.0f; mgr.update(now, 0.0f, mass, true);
    }
    TEST_ASSERT_EQUAL(CalibrationManager::State::AWAIT_BLIND_BASKET, mgr.getState());
    TEST_ASSERT_TRUE(mgr.confirmBlindBasket());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_STALL, mgr.getState());

    // Point 0 starts normally
    mgr.update(now, 4.0f, mass, true);
    now += 500;
    mgr.update(now, 4.2f, mass, true);

    // Sudden dangerous pressure spike (12.5 bar > 12.0 bar limit)
    now += 100;
    mgr.update(now, 12.5f, mass, true);

    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());
    TEST_ASSERT_NOT_NULL(strstr(mgr.getLastError(), "12 bar"));
}

void test_adversarial_calibration_dry_tank_zero_mass() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    uint32_t now = 1000;
    float mass = 50.0f;

    // Step 0 (20% duty): scale mass does not increase (tank dry or empty)
    mgr.update(now, 0.0f, mass, true);
    now += 1000; mgr.update(now, 0.0f, mass, true);
    now += 4000; mgr.update(now, 0.0f, mass, true);

    // Step 0 completes with 0 mass delta (Q clamped to 0.1)
    TEST_ASSERT_EQUAL_UINT(1, mgr.getCurrentPointIndex());
    TEST_ASSERT_EQUAL_FLOAT(40.0f, mgr.getActiveDutyCommand());

    // Step 1 (40% duty): scale mass still does not increase (tank is definitely dry)
    mgr.update(now, 0.0f, mass, true);
    now += 1000; mgr.update(now, 0.0f, mass, true);
    now += 4000; mgr.update(now, 0.0f, mass, true);

    // At 40% duty with deltaM < 1.0g, CalibrationManager must abort with dry tank error
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mgr.getActiveDutyCommand());
    TEST_ASSERT_FALSE(mgr.isValveOpenCommand());
    TEST_ASSERT_NOT_NULL(strstr(mgr.getLastError(), "water tank empty"));
}

void test_adversarial_calibration_non_monotonic_rejection() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    // Complete Open Flow with valid numbers
    uint32_t now = 1000;
    float mass = 10.0f;
    for (size_t pt = 0; pt < 5; ++pt) {
        mgr.update(now, 0.0f, mass, true);
        now += 1000; mass += 2.0f; mgr.update(now, 0.0f, mass, true);
        now += 4000; mass += 10.0f; mgr.update(now, 0.0f, mass, true);
    }
    TEST_ASSERT_TRUE(mgr.confirmBlindBasket());

    // In stall sweep, inject non-monotonic pressures:
    // Pt 0 (20%): 8.0 bar
    // Pt 1 (40%): 5.0 bar (DROPPED! Hydraulic leak or sensor error)
    // Pt 2 (60%): 9.0 bar
    // Pt 3 (80%): 10.0 bar
    // Pt 4 (100%): 11.0 bar
    const float nonMonotonicP[5] = {8.0f, 5.0f, 9.0f, 10.0f, 11.0f};

    for (size_t pt = 0; pt < 5; ++pt) {
        mgr.update(now, nonMonotonicP[pt], mass, true);
        now += 1000;
        mgr.update(now, nonMonotonicP[pt], mass, true);
        now += 4000;
        mgr.update(now, nonMonotonicP[pt], mass, true);
    }

    // Must detect non-monotonic candidate, abort, and restore factory defaults
    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_FALSE(mgr.isCalibrated());
    TEST_ASSERT_NOT_NULL(strstr(mgr.getLastError(), "validation failed"));

    // Verify model in manager restored factory calibration
    TEST_ASSERT_EQUAL_FLOAT(4.0f, mgr.getModel().getStallPressure(20.0f));
    TEST_ASSERT_EQUAL_FLOAT(14.0f, mgr.getModel().getStallPressure(100.0f));
}

void test_adversarial_calibration_safety_timeout() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    uint32_t now = 1000;
    mgr.update(now, 0.0f, 10.0f, true);

    // Advance time by 65 seconds without completing open flow sweep
    now += 65000;
    mgr.update(now, 0.0f, 10.0f, true);

    TEST_ASSERT_EQUAL(CalibrationManager::State::ERROR_ABORT, mgr.getState());
    TEST_ASSERT_NOT_NULL(strstr(mgr.getLastError(), "safety timeout"));
}

void test_adversarial_sensor_disconnect_and_nan_invariants() {
    PumpSurfaceModel model;
    const float nanVal = std::numeric_limits<float>::quiet_NaN();
    const float infVal = std::numeric_limits<float>::infinity();

    // Verify for a wide range of flow targets (0.1 to 10.0 ml/s) that NaN pressure NEVER causes runaway
    const float targetFlows[] = {0.1f, 0.5f, 1.0f, 2.0f, 2.5f, 3.5f, 5.0f, 7.5f, 8.5f, 10.0f};
    for (float qTarget : targetFlows) {
        float duty = model.getFeedforwardDuty(qTarget, nanVal, 1.0f);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, duty, "Feedforward duty on NaN pressure must be 0.0% (fail-safe shutoff)");
    }

    // Verify for a wide range of pressures (0.0 to 15.0 bar) that NaN target flow returns 0.0%
    const float testPressures[] = {0.0f, 1.0f, 3.0f, 6.0f, 9.0f, 12.0f, 15.0f};
    for (float P : testPressures) {
        float duty = model.getFeedforwardDuty(nanVal, P, 1.0f);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, duty, "Feedforward duty on NaN target flow must be 0.0%");
    }

    // Verify NaN and Inf in thermal scalar
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 4.0f, nanVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 4.0f, infVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, 4.0f, -infVal));

    // Verify Inf in pressure and target flow
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(infVal, 4.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(-infVal, 4.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFeedforwardDuty(2.5f, infVal, 1.0f));

    // interpolateNodalVector via getMaxFlow / getStallPressure with NaN and Inf
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getMaxFlow(nanVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getStallPressure(nanVal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(nanVal, 50.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, model.getFlow(5.0f, nanVal));
}

void test_adversarial_strict_monotonicity_and_thresholds() {
    PumpCalibrationData cal;

    // 1. Strict monotonicity on dutyPoints: any adjacent equal or decreasing node must be rejected
    for (size_t i = 1; i < PumpCalibrationData::NUM_POINTS; ++i) {
        cal = PumpCalibrationData();
        cal.dutyPoints[i] = cal.dutyPoints[i - 1]; // Equal
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Equal adjacent duty nodes must be rejected");

        cal = PumpCalibrationData();
        cal.dutyPoints[i] = cal.dutyPoints[i - 1] - 0.1f; // Decreasing
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Decreasing adjacent duty nodes must be rejected");
    }

    // 2. Strict monotonicity on qMaxPoints
    for (size_t i = 1; i < PumpCalibrationData::NUM_POINTS; ++i) {
        cal = PumpCalibrationData();
        cal.qMaxPoints[i] = cal.qMaxPoints[i - 1]; // Equal
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Equal adjacent qMax nodes must be rejected");

        cal = PumpCalibrationData();
        cal.qMaxPoints[i] = cal.qMaxPoints[i - 1] - 0.05f; // Decreasing
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Decreasing adjacent qMax nodes must be rejected");
    }

    // 3. Strict monotonicity on pStallPoints
    for (size_t i = 1; i < PumpCalibrationData::NUM_POINTS; ++i) {
        cal = PumpCalibrationData();
        cal.pStallPoints[i] = cal.pStallPoints[i - 1]; // Equal
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Equal adjacent pStall nodes must be rejected");

        cal = PumpCalibrationData();
        cal.pStallPoints[i] = cal.pStallPoints[i - 1] - 0.1f; // Decreasing
        TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "Decreasing adjacent pStall nodes must be rejected");
    }

    // 4. Stall pressure threshold boundaries: pStallPoints[4] in [6.0, 20.0]
    cal = PumpCalibrationData();
    cal.pStallPoints[0] = 1.0f;
    cal.pStallPoints[1] = 2.0f;
    cal.pStallPoints[2] = 3.0f;
    cal.pStallPoints[3] = 4.0f;
    cal.pStallPoints[4] = 5.99f;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "pStallPoints[4] < 6.0 must be rejected");

    cal.pStallPoints[4] = 6.0f;
    TEST_ASSERT_TRUE_MESSAGE(cal.isValid(), "pStallPoints[4] == 6.0 must be valid");

    cal.pStallPoints[4] = 20.0f;
    TEST_ASSERT_TRUE_MESSAGE(cal.isValid(), "pStallPoints[4] == 20.0 must be valid");

    cal.pStallPoints[4] = 20.01f;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "pStallPoints[4] > 20.0 must be rejected");

    // 5. Open flow threshold boundaries: qMaxPoints[4] in [3.0, 25.0]
    cal = PumpCalibrationData();
    cal.qMaxPoints[0] = 0.5f;
    cal.qMaxPoints[1] = 1.0f;
    cal.qMaxPoints[2] = 1.5f;
    cal.qMaxPoints[3] = 2.0f;
    cal.qMaxPoints[4] = 2.99f;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "qMaxPoints[4] < 3.0 must be rejected");

    cal.qMaxPoints[4] = 3.0f;
    TEST_ASSERT_TRUE_MESSAGE(cal.isValid(), "qMaxPoints[4] == 3.0 must be valid");

    cal.qMaxPoints[4] = 25.0f;
    TEST_ASSERT_TRUE_MESSAGE(cal.isValid(), "qMaxPoints[4] == 25.0 must be valid");

    cal.qMaxPoints[4] = 25.01f;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "qMaxPoints[4] > 25.0 must be rejected");
}

void test_adversarial_full_calibration_sweep_across_millis_rollover() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    // Start 15000 ms before 32-bit rollover: 0xFFFFC568 (-15000)
    // Entire 2-step sweep will take 5 * 5s + swap + 5 * 5s = ~55s, passing across 0x00000000
    uint32_t now = 0xFFFFC568U;
    float mass = 100.0f;
    const float syntheticQmax[5] = {1.9f, 3.6f, 5.4f, 7.1f, 8.4f};

    // Open Flow sweep across rollover
    for (size_t pt = 0; pt < 5; ++pt) {
        TEST_ASSERT_EQUAL_STRING_MESSAGE("SWEEP_OPEN_FLOW", mgr.getStateString(), mgr.getLastError());
        TEST_ASSERT_EQUAL_UINT(pt, mgr.getCurrentPointIndex());
        mgr.update(now, 0.0f, mass, true);

        // Progress should be non-decreasing and bounded
        float p0 = mgr.getStepProgressPercent(now);
        TEST_ASSERT_TRUE(p0 >= 0.0f && p0 <= 100.0f);

        now += CalibrationManager::SETTLE_DWELL_MS;
        mass += syntheticQmax[pt] * 1.0f;
        mgr.update(now, 0.0f, mass, true);

        float p1 = mgr.getStepProgressPercent(now);
        TEST_ASSERT_TRUE(p1 >= p0 && p1 <= 100.0f);

        const uint32_t dtMs = CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS;
        now += dtMs;
        mass += syntheticQmax[pt] * (static_cast<float>(dtMs) / 1000.0f);
        mgr.update(now, 0.0f, mass, true);
    }

    TEST_ASSERT_EQUAL(CalibrationManager::State::AWAIT_BLIND_BASKET, mgr.getState());
    TEST_ASSERT_TRUE(mgr.confirmBlindBasket());
    TEST_ASSERT_EQUAL(CalibrationManager::State::SWEEP_STALL, mgr.getState());

    // Stall pressure sweep across rollover
    const float syntheticPstall[5] = {4.1f, 6.6f, 9.4f, 11.2f, 11.9f};
    for (size_t pt = 0; pt < 5; ++pt) {
        TEST_ASSERT_EQUAL_UINT(pt, mgr.getCurrentPointIndex());
        mgr.update(now, syntheticPstall[pt], mass, true);

        now += CalibrationManager::SETTLE_DWELL_MS;
        mgr.update(now, syntheticPstall[pt], mass, true);

        const uint32_t dtMs = CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS;
        now += dtMs;
        mgr.update(now, syntheticPstall[pt], mass, true);
    }

    TEST_ASSERT_EQUAL(CalibrationManager::State::SAVED, mgr.getState());
    TEST_ASSERT_TRUE(mgr.isCalibrated());

    // Verify calibration data was correctly recorded across rollover
    const auto &cal = mgr.getCalibratedData();
    for (size_t i = 0; i < 5; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(0.05f, syntheticQmax[i], cal.qMaxPoints[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, syntheticPstall[i], cal.pStallPoints[i]);
    }
}

void test_adversarial_compliance_and_thermal_fade_nan_inf() {
    PumpCalibrationData cal;
    const float nanVal = std::numeric_limits<float>::quiet_NaN();
    const float infVal = std::numeric_limits<float>::infinity();

    // System compliance boundary checks: [0.2, 5.0]
    cal = PumpCalibrationData();
    cal.systemCompliance = 0.19f;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = 0.20f;
    TEST_ASSERT_TRUE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = 5.00f;
    TEST_ASSERT_TRUE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = 5.01f;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = infVal;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = -infVal;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.systemCompliance = nanVal;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "systemCompliance == NaN must be rejected by isValid()");

    // Thermal fade factor boundary checks: [0.0, 0.5]
    cal = PumpCalibrationData();
    cal.thermalFadeFactor = -0.01f;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = 0.00f;
    TEST_ASSERT_TRUE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = 0.50f;
    TEST_ASSERT_TRUE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = 0.51f;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = infVal;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = -infVal;
    TEST_ASSERT_FALSE(cal.isValid());

    cal = PumpCalibrationData();
    cal.thermalFadeFactor = nanVal;
    TEST_ASSERT_FALSE_MESSAGE(cal.isValid(), "thermalFadeFactor == NaN must be rejected by isValid()");
}

void test_adversarial_rollover_sentinel_collision_at_zero() {
    CalibrationManager mgr;
    TEST_ASSERT_TRUE(mgr.startCalibration());

    // Step 0 starts at 0xFFFFEC78 (-5000 ms)
    uint32_t now = 0xFFFFEC78U;
    mgr.update(now, 0.0f, 10.0f, true);
    now += 1000;
    mgr.update(now, 0.0f, 12.0f, true);
    // Step 0 reaches 5000 ms at exactly now = 0x00000000
    now += 4000; // now is exactly 0x00000000!
    mgr.update(now, 0.0f, 20.0f, true);

    // Step 0 finished and advanced to step 1
    TEST_ASSERT_EQUAL_UINT(1, mgr.getCurrentPointIndex());

    // Next tick is at now = 1000 ms (settle dwell)
    now += CalibrationManager::SETTLE_DWELL_MS;
    mgr.update(now, 0.0f, 23.5f, true);

    // Next tick is at now = 5000 ms (step dwell complete: 5000ms from start)
    now += (CalibrationManager::STEP_DWELL_MS - CalibrationManager::SETTLE_DWELL_MS);
    mgr.update(now, 0.0f, 35.0f, true);

    // Step 1 MUST advance to step 2 after 5000 ms elapsed
    TEST_ASSERT_EQUAL_UINT_MESSAGE(2, mgr.getCurrentPointIndex(),
        "Step 1 must finish after 5000ms elapsed even when its start timestamp was 0x00000000");
}

// ===========================================================================
// Unity Main Runner
// ===========================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_adversarial_extreme_pressure_clamping);
    RUN_TEST(test_adversarial_extreme_duty_cycles);
    RUN_TEST(test_adversarial_monotonicity_fine_grid);
    RUN_TEST(test_adversarial_feedforward_inversion_boundary_matrix);
    RUN_TEST(test_adversarial_thermal_fade_clamping);
    RUN_TEST(test_adversarial_calibration_rapid_aborts);
    RUN_TEST(test_adversarial_calibration_mid_sweep_scale_disconnects);
    RUN_TEST(test_adversarial_calibration_runaway_pressure_spike);
    RUN_TEST(test_adversarial_calibration_dry_tank_zero_mass);
    RUN_TEST(test_adversarial_calibration_non_monotonic_rejection);
    RUN_TEST(test_adversarial_calibration_safety_timeout);
    RUN_TEST(test_adversarial_sensor_disconnect_and_nan_invariants);
    RUN_TEST(test_adversarial_strict_monotonicity_and_thresholds);
    RUN_TEST(test_adversarial_full_calibration_sweep_across_millis_rollover);
    RUN_TEST(test_adversarial_compliance_and_thermal_fade_nan_inf);
    RUN_TEST(test_adversarial_rollover_sentinel_collision_at_zero);

    return UNITY_END();
}

