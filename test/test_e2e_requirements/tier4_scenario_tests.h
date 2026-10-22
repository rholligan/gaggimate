// Tier 4: Real-World Application Scenarios (5 Authentic Espresso Profiles)
// Validates end-to-end multi-stage profiles against physical hydraulic & thermal models
#pragma once

#include <unity.h>
#include "../common/hydraulic_mocks.h"

using namespace gaggimate_test;

// ============================================================================
// Tier 4: Real-World Espresso Profiles
// ============================================================================

// Profile 1: Classic 9 Bar Italian Espresso
// Stage 1 (Pre-infusion): 3.0 bar for 4s
// Stage 2 (Extraction): 9.0 bar flat hold
// Stage 3 (Auto-stop): Predictive cutoff targeting 36.0g (+/- 0.3g)
static void test_r4_scenario_classic_nine_bar_espresso() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f);
    SyntheticScaleMock scale(100, 70); // 100ms BLE delay, 70ms valve latency
    scale.reset();
    plant.reset(0.0f);
    plant.setPuckResistance(3.6f); // Italian espresso grind

    float targetMass = 36.0f;
    uint32_t now = 1000;
    float currentDuty = 25.0f;
    bool cutoffTriggered = false;

    // Fast-forward simulation (dt = 0.03s)
    for (int step = 0; step < 1000; ++step) {
        now += 30;
        float p = plant.getPressure();

        // Profile target arbitration
        if (now < 5000) {
            // Stage 1: Pre-infusion 3.0 bar
            currentDuty = (p < 3.0f) ? 35.0f : 20.0f;
        } else {
            // Stage 2: 9.0 bar flat hold
            currentDuty = (p < 9.0f) ? 60.0f : 48.0f;
        }

        float qPump = pump.getFlow(p, currentDuty);
        auto state = plant.step(qPump, 0.03f);

        // Water reaches cup if valve open
        scale.addPuckFlow(state.puckFlow, 0.03f, now);

        // Predictive auto-stop check
        if (scale.shouldAutoStop(targetMass, now)) {
            cutoffTriggered = true;
            // Synchronized cutoff: valve close, pump off
            currentDuty = 0.0f;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(cutoffTriggered, "Classic 9 bar profile must trigger auto-stop");
    // Rested cup weight: ground truth mass plus falling stream within +/- 0.3g
    float restedMass = scale.getGroundTruthMass();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.30f, targetMass, restedMass,
                                     "Classic 9 bar profile must land on 36.0g +/- 0.3g");
}

// Profile 2: Blooming Espresso Profile
// Stage 1 (Fast Fill): 4.0 ml/s high flow fill for 4.0s
// Stage 2 (Bloom): Zero flow (0.0 ml/s, duty 0%) for 20.0s
// Stage 3 (Extraction): 6.0 bar gentle extraction tapering to 2.0 ml/s
// Stage 4 (Auto-stop): Cutoff targeting 40.0g (+/- 0.3g)
static void test_r4_scenario_blooming_espresso_profile() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f);
    SyntheticScaleMock scale(100, 70);
    FlowPIDController pid(4.0f, 2.0f, 0.03f);

    scale.reset();
    plant.reset(0.0f);
    plant.setPuckResistance(2.8f);

    float targetMass = 40.0f;
    uint32_t now = 1000;
    float currentDuty = 0.0f;
    bool cutoffTriggered = false;

    for (int step = 0; step < 1500; ++step) {
        now += 30;
        float elapsedSec = (now - 1000) / 1000.0f;

        if (elapsedSec < 4.0f) {
            // Stage 1: Fast fill at 4.0 ml/s
            currentDuty = 55.0f;
        } else if (elapsedSec < 24.0f) {
            // Stage 2: Bloom pause at 0 ml/s
            currentDuty = 0.0f;
            pid.reset(); // Zero windup during bloom
        } else {
            // Stage 3: Extraction at 2.2 ml/s flow target
            float targetFlow = 2.2f;
            float ff = pump.getFeedforwardDuty(targetFlow, plant.getPressure());
            currentDuty = pid.update(targetFlow, plant.getPressure() / plant.getPuckResistance(), ff);
        }

        float qPump = pump.getFlow(plant.getPressure(), currentDuty);
        auto state = plant.step(qPump, 0.03f);

        if (elapsedSec >= 4.0f) {
            scale.addPuckFlow(state.puckFlow, 0.03f, now);
        }

        if (scale.shouldAutoStop(targetMass, now)) {
            cutoffTriggered = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(cutoffTriggered, "Blooming profile must trigger auto-stop");
    float restedMass = scale.getGroundTruthMass();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.30f, targetMass, restedMass,
                                     "Blooming profile must hit 40.0g +/- 0.3g");
}

// Profile 3: Turbo Shot (High Flow, Low Pressure)
// Stage 1 (Fast Ramp): 6.0 bar target with coarse grind (R_puck = 1.4)
// Stage 2 (High Flow Hold): Constant 4.2 ml/s flow hold
// Stage 3 (Fast Cutoff): Auto-stop targeting 45.0g within 16s total duration
static void test_r4_scenario_turbo_shot_high_flow_low_pressure() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f);
    SyntheticScaleMock scale(80, 60);
    scale.reset();
    scale.setLeadTime(0.12f); // Calibrated lead time for fast flow
    plant.reset(0.0f);
    plant.setPuckResistance(1.4f); // Coarse turbo grind

    float targetMass = 45.0f;
    uint32_t now = 1000;
    float currentDuty = 65.0f;
    bool cutoffTriggered = false;

    for (int step = 0; step < 700; ++step) {
        now += 30;
        float p = plant.getPressure();

        // Control high flow 4.2 ml/s
        float qPump = pump.getFlow(p, currentDuty);
        auto state = plant.step(qPump, 0.03f);
        scale.addPuckFlow(state.puckFlow, 0.03f, now);

        if (scale.shouldAutoStop(targetMass, now)) {
            cutoffTriggered = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(cutoffTriggered, "Turbo shot must trigger cutoff");
    float totalTimeSec = (now - 1000) / 1000.0f;
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(16.0f, totalTimeSec, "Turbo shot should finish in under 16 seconds");
    float restedMass = scale.getGroundTruthMass();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.30f, targetMass, restedMass,
                                     "Turbo shot yield must hit 45.0g +/- 0.3g");
}

// Profile 4: Slayer-Style Slow Flow Profile
// Stage 1 (Slow Pre-wetting): Flow restricted to 1.5 ml/s until pressure hits 4.0 bar
// Stage 2 (Full Pressure): Ramp to 9.0 bar hold until 24.0g
// Stage 3 (Flow Taper): Decline flow to 1.8 ml/s until 38.0g
// Stage 4 (Synchronized Cutoff): Valve vented and pump zeroed
static void test_r4_scenario_slayer_style_flow_profile() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f);
    SyntheticScaleMock scale(100, 70);
    scale.reset();
    plant.reset(0.0f);
    plant.setPuckResistance(3.4f);

    float targetMass = 38.0f;
    uint32_t now = 1000;
    float currentDuty = 25.0f;
    bool cutoffTriggered = false;

    for (int step = 0; step < 1200; ++step) {
        now += 30;
        float p = plant.getPressure();
        float m = scale.getReportedMass();

        if (p < 4.0f && m < 2.0f) {
            // Stage 1: Slow pre-wetting at 1.5 ml/s
            currentDuty = 28.0f;
        } else if (m < 24.0f) {
            // Stage 2: Full 9.0 bar extraction
            currentDuty = (p < 9.0f) ? 58.0f : 50.0f;
        } else {
            // Stage 3: Flow taper to 1.8 ml/s
            currentDuty = 38.0f;
        }

        float qPump = pump.getFlow(p, currentDuty);
        auto state = plant.step(qPump, 0.03f);
        scale.addPuckFlow(state.puckFlow, 0.03f, now);

        if (scale.shouldAutoStop(targetMass, now)) {
            cutoffTriggered = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(cutoffTriggered, "Slayer profile must trigger auto-stop");
    float restedMass = scale.getGroundTruthMass();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.30f, targetMass, restedMass,
                                     "Slayer profile must hit 38.0g +/- 0.3g");
}

// Profile 5: Lever Declining Pressure Profile
// Stage 1 (Pre-infusion): 2.0 bar for 6s
// Stage 2 (Peak Pressure): Instantaneous rise to 9.0 bar peak
// Stage 3 (Spring Decay): Linear pressure decline from 9.0 bar down to 5.5 bar as puck dissolves
// Stage 4 (Cutoff): Predictive auto-stop targeting 34.0g (+/- 0.3g)
static void test_r4_scenario_lever_declining_pressure_profile() {
    UlkaPumpPhysicsModel pump;
    HydraulicCircuitSimulator plant(1.40f);
    SyntheticScaleMock scale(100, 70);
    scale.reset();
    plant.reset(0.0f);
    plant.setPuckResistance(3.5f);

    float targetMass = 34.0f;
    uint32_t now = 1000;
    float currentDuty = 22.0f;
    bool cutoffTriggered = false;

    for (int step = 0; step < 1000; ++step) {
        now += 30;
        float elapsedSec = (now - 1000) / 1000.0f;
        float p = plant.getPressure();

        if (elapsedSec < 6.0f) {
            // Stage 1: 2.0 bar gentle pre-infusion
            currentDuty = (p < 2.0f) ? 25.0f : 18.0f;
        } else {
            // Stage 3: Declining spring pressure (9.0 down to 5.5 bar)
            float postPreSec = elapsedSec - 6.0f;
            float targetP = std::max(5.5f, 9.0f - (postPreSec * 0.175f));
            currentDuty = (p < targetP) ? 55.0f : 42.0f;
        }

        float qPump = pump.getFlow(p, currentDuty);
        auto state = plant.step(qPump, 0.03f);
        scale.addPuckFlow(state.puckFlow, 0.03f, now);

        if (scale.shouldAutoStop(targetMass, now)) {
            cutoffTriggered = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(cutoffTriggered, "Lever declining pressure profile must trigger auto-stop");
    float restedMass = scale.getGroundTruthMass();
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.30f, targetMass, restedMass,
                                     "Lever declining profile must hit 34.0g +/- 0.3g");
}
