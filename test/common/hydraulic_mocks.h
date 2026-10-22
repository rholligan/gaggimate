// Independent physical plant mocks and opaque-box test fixtures for GaggiMate E2E testing.
// Implements authoritative physical models according to spec_inventory.md §3 and ORIGINAL_REQUEST.md.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace gaggimate_test {

// ============================================================================
// 1. Physical Ulka Linear Displacement Pump Model
// Q(P, Duty) = Qmax(Duty) * (1 - P / Pstall(Duty)) * S_thermal(Theta)
// ============================================================================
struct PumpCalibrationData {
    static constexpr size_t NUM_POINTS = 5;
    float dutyPoints[NUM_POINTS] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
    float qMaxPoints[NUM_POINTS] = {1.80f, 3.50f, 5.50f, 7.20f, 8.50f};      // ml/s at P=0
    float pStallPoints[NUM_POINTS] = {4.00f, 6.50f, 9.50f, 12.00f, 14.00f};  // bar at Q=0
    float systemCompliance = 1.40f;                                           // ml/bar
    float thermalFadeFactor = 0.15f;                                          // kappa_fade
};

class UlkaPumpPhysicsModel {
public:
    UlkaPumpPhysicsModel() {
        cal_ = PumpCalibrationData{};
    }

    explicit UlkaPumpPhysicsModel(const PumpCalibrationData &cal) : cal_(cal) {}

    void setCalibration(const PumpCalibrationData &cal) { cal_ = cal; }
    const PumpCalibrationData &getCalibration() const { return cal_; }

    float getMaxFlow(float dutyPercent, float thermalScalar = 1.0f) const {
        if (dutyPercent < 15.0f) return 0.0f;
        float qMax = interpolateNode(dutyPercent, cal_.qMaxPoints);
        return std::max(0.0f, qMax * thermalScalar);
    }

    float getStallPressure(float dutyPercent, float thermalScalar = 1.0f) const {
        if (dutyPercent < 15.0f) return 0.0f;
        float pStall = interpolateNode(dutyPercent, cal_.pStallPoints);
        return std::max(0.0f, pStall * thermalScalar);
    }

    // Forward displacement model
    float getFlow(float pressureBar, float dutyPercent, float thermalScalar = 1.0f) const {
        if (dutyPercent < 15.0f) return 0.0f;
        if (pressureBar < 0.0f) pressureBar = 0.0f;

        float qMax = getMaxFlow(dutyPercent, thermalScalar);
        float pStall = getStallPressure(dutyPercent, thermalScalar);

        if (pStall <= 0.001f || pressureBar >= pStall) {
            return 0.0f;
        }

        float flow = qMax * (1.0f - pressureBar / pStall);
        return std::max(0.0f, flow);
    }

    // Inverted feedforward duty solver: solves f(P, duty) = targetFlow via bisection
    float getFeedforwardDuty(float targetFlowMlS, float pressureBar, float thermalScalar = 1.0f) const {
        if (targetFlowMlS <= 0.001f) return 0.0f;

        float low = 15.0f;
        float high = 100.0f;
        float bestDuty = 0.0f;

        // Verify if achievable at maximum duty
        float flowAtMax = getFlow(pressureBar, 100.0f, thermalScalar);
        if (targetFlowMlS >= flowAtMax) {
            return 100.0f;
        }

        // 12-iteration bisection solves duty within ~0.02%
        for (int i = 0; i < 14; ++i) {
            float mid = 0.5f * (low + high);
            float fMid = getFlow(pressureBar, mid, thermalScalar);
            if (fMid < targetFlowMlS) {
                low = mid;
                bestDuty = mid;
            } else {
                high = mid;
                bestDuty = mid;
            }
        }
        return std::clamp(bestDuty, 0.0f, 100.0f);
    }

private:
    PumpCalibrationData cal_;

    float interpolateNode(float duty, const float *nodes) const {
        if (duty <= cal_.dutyPoints[0]) return nodes[0];
        if (duty >= cal_.dutyPoints[PumpCalibrationData::NUM_POINTS - 1]) {
            return nodes[PumpCalibrationData::NUM_POINTS - 1];
        }

        for (size_t i = 0; i < PumpCalibrationData::NUM_POINTS - 1; ++i) {
            if (duty >= cal_.dutyPoints[i] && duty <= cal_.dutyPoints[i + 1]) {
                float alpha = (duty - cal_.dutyPoints[i]) / (cal_.dutyPoints[i + 1] - cal_.dutyPoints[i]);
                return (1.0f - alpha) * nodes[i] + alpha * nodes[i + 1];
            }
        }
        return nodes[0];
    }
};

// ============================================================================
// 2. Solenoid Coil Thermal Energy Accumulator Mock
// dTheta/dt = k_heat * (Duty/100)^2 - k_cool * Theta
// ============================================================================
class ThermalAccumulatorMock {
public:
    static constexpr float kHeat = 0.0167f;  // s^-1 (heats in ~60s at 100% duty)
    static constexpr float kCool = 0.0040f;  // s^-1 (tau_cool = 250s)
    static constexpr float kFade = 0.1500f;  // 15% maximum degradation

    explicit ThermalAccumulatorMock(float initialTheta = 0.0f)
        : theta_(std::clamp(initialTheta, 0.0f, 1.0f)) {}

    void reset(float initialTheta = 0.0f) {
        theta_ = std::clamp(initialTheta, 0.0f, 1.0f);
    }

    void update(float dutyPercent, float dtSeconds) {
        if (dtSeconds <= 0.0f) return;
        float u = std::clamp(dutyPercent / 100.0f, 0.0f, 1.0f);
        float decay = std::exp(-kCool * dtSeconds);
        float steadyState = (kHeat * u * u) / kCool;
        theta_ = std::clamp(theta_ * decay + steadyState * (1.0f - decay), 0.0f, 1.0f);
    }

    float getThermalState() const { return theta_; }

    // S_thermal = 1.0 - kappa_fade * Theta
    float getThermalScalar() const {
        float scalar = 1.0f - kFade * theta_;
        return std::clamp(scalar, 0.80f, 1.0f);
    }

    // Feedforward duty boost: Duty_comp = Duty_nom / S_thermal
    float compensateDuty(float nominalDuty) const {
        float scalar = getThermalScalar();
        if (scalar <= 0.01f) return nominalDuty;
        return std::clamp(nominalDuty / scalar, 0.0f, 100.0f);
    }

private:
    float theta_ = 0.0f;
};

// ============================================================================
// 3. Hydraulic Circuit & Darcy Puck Resistance Plant Mock
// dP/dt = (Q_pump - Q_puck) / C_sys
// Q_puck = P / R_puck
// ============================================================================
class HydraulicCircuitSimulator {
public:
    HydraulicCircuitSimulator(float complianceMlPerBar = 1.40f, float voidVolumeMl = 8.0f)
        : compliance_(complianceMlPerBar), voidVolume_(voidVolumeMl) {}

    void reset(float initialPressure = 0.0f) {
        pressure_ = initialPressure;
        absorbedWater_ = 0.0f;
        saturated_ = false;
        filteredPressureDeriv_ = 0.0f;
        puckResistance_ = 3.5f; // default medium-fine resistance
    }

    void setPuckResistance(float rPuckBarSPerMl) {
        puckResistance_ = std::max(0.01f, rPuckBarSPerMl);
    }

    float getPuckResistance() const { return puckResistance_; }
    float getPressure() const { return pressure_; }
    bool isSaturated() const { return saturated_; }

    // Advances plant physics by dt seconds given pump output flow
    struct State {
        float pressure;       // bar
        float pumpFlow;       // ml/s
        float puckFlow;       // ml/s
        float complianceFlow;// ml/s
        float dPdt;          // bar/s
    };

    State step(float pumpDisplacementFlow, float dtSeconds) {
        if (dtSeconds <= 0.0f) dtSeconds = 0.03f;

        float qPuck = 0.0f;
        if (!saturated_) {
            // Pre-infusion void filling: low puck resistance, absorbing water
            absorbedWater_ += pumpDisplacementFlow * dtSeconds;
            if (absorbedWater_ >= voidVolume_) {
                saturated_ = true;
            }
            qPuck = pressure_ / (puckResistance_ * 5.0f); // loose coffee bed
        } else {
            // Saturated extraction: Darcy flow
            qPuck = pressure_ / puckResistance_;
        }

        // Compliance pressure evolution: dP/dt = (Q_pump - Q_puck) / C
        float dPdtRaw = (pumpDisplacementFlow - qPuck) / compliance_;

        // Low-pass filter on dP/dt to emulate Kalman derivative filter (tau = 0.1s)
        float alpha = dtSeconds / (dtSeconds + 0.10f);
        filteredPressureDeriv_ += alpha * (dPdtRaw - filteredPressureDeriv_);

        // Euler integration of pressure
        pressure_ += dPdtRaw * dtSeconds;
        if (pressure_ < 0.0f) pressure_ = 0.0f;

        float complianceFlow = compliance_ * filteredPressureDeriv_;
        float estimatedPuckFlow = std::max(0.0f, pumpDisplacementFlow - complianceFlow);

        return State{pressure_, pumpDisplacementFlow, estimatedPuckFlow, complianceFlow, filteredPressureDeriv_};
    }

private:
    float compliance_ = 1.40f;
    float voidVolume_ = 8.00f;
    float pressure_ = 0.0f;
    float absorbedWater_ = 0.0f;
    bool saturated_ = false;
    float filteredPressureDeriv_ = 0.0f;
    float puckResistance_ = 3.5f;
};

// ============================================================================
// 4. Synthetic BLE Scale Mock with Latency Queue & In-Flight Stream
// ============================================================================
class SyntheticScaleMock {
public:
    SyntheticScaleMock(uint32_t transportDelayMs = 100, uint32_t valveDelayMs = 70)
        : transportDelayMs_(transportDelayMs), valveDelayMs_(valveDelayMs) {}

    void reset() {
        groundTruthMass_ = 0.0f;
        packetQueue_.clear();
        sampleHistory_.clear();
        tLeadSec_ = 0.20f;
    }

    void setLeadTime(float leadTimeSec) { tLeadSec_ = leadTimeSec; }
    float getLeadTime() const { return tLeadSec_; }

    // Adds true liquid entering the cup
    void addPuckFlow(float puckFlowMlS, float dtSeconds, uint32_t nowMs) {
        if (dtSeconds <= 0.0f) return;
        groundTruthMass_ += puckFlowMlS * dtSeconds; // 1 ml ~ 1 g

        // Push into latency buffer (simulating BLE connection interval & transport)
        packetQueue_.push_back(Packet{nowMs + transportDelayMs_, groundTruthMass_});

        // Maintain 800ms sliding regression history of received samples
        while (!packetQueue_.empty() && packetQueue_.front().deliverAtMs <= nowMs) {
            sampleHistory_.push_back(packetQueue_.front());
            packetQueue_.pop_front();
        }

        while (!sampleHistory_.empty() && (nowMs - sampleHistory_.front().deliverAtMs) > 800) {
            sampleHistory_.pop_front();
        }
    }

    float getReportedMass() const {
        if (sampleHistory_.empty()) return 0.0f;
        return sampleHistory_.back().mass;
    }

    float getGroundTruthMass() const { return groundTruthMass_; }

    // Linear regression mass rate dm/dt over sliding window
    float getMassRate(uint32_t nowMs) const {
        if (sampleHistory_.size() < 2) return 0.0f;

        double sumT = 0.0;
        double sumM = 0.0;
        size_t n = sampleHistory_.size();

        for (const auto &p : sampleHistory_) {
            sumT += p.deliverAtMs;
            sumM += p.mass;
        }

        double meanT = sumT / n;
        double meanM = sumM / n;

        double covTM = 0.0;
        double varT = 0.0;

        for (const auto &p : sampleHistory_) {
            double dt = p.deliverAtMs - meanT;
            double dm = p.mass - meanM;
            covTM += dt * dm;
            varT += dt * dt;
        }

        if (varT < 1e-6) return 0.0f;
        double slopeGPerMs = covTM / varT;
        return static_cast<float>(std::max(0.0, slopeGPerMs * 1000.0)); // g/s
    }

    // Predictive auto-stop criterion: m_predicted = m + (dm/dt) * t_lead >= m_target
    bool shouldAutoStop(float targetMassGrams, uint32_t nowMs) const {
        float m = getReportedMass();
        float rate = getMassRate(nowMs);
        float predicted = m + rate * tLeadSec_;
        return predicted >= targetMassGrams;
    }

    // Post-shot adaptive learning of lead time
    float updateLearnedLeadTime(float targetMassGrams, float finalRestingMassGrams, float lastCutoffRateGPerS) {
        if (lastCutoffRateGPerS < 0.5f) return tLeadSec_;
        float deltaM = finalRestingMassGrams - targetMassGrams;
        // Outlier rejection (e.g. cup grabbed prematurely)
        if (std::abs(deltaM) > 10.0f) return tLeadSec_;

        float eta = 0.30f; // learning rate
        float adjustment = eta * (deltaM / lastCutoffRateGPerS);
        tLeadSec_ = std::clamp(tLeadSec_ + adjustment, 0.05f, 0.60f);
        return tLeadSec_;
    }

private:
    struct Packet {
        uint32_t deliverAtMs;
        float mass;
    };

    uint32_t transportDelayMs_;
    uint32_t valveDelayMs_;
    float groundTruthMass_ = 0.0f;
    float tLeadSec_ = 0.20f;
    std::deque<Packet> packetQueue_;
    std::deque<Packet> sampleHistory_;
};

// ============================================================================
// 5. Dynamic Closed-Loop Flow PID Controller with Bumpless Handover
// ============================================================================
class FlowPIDController {
public:
    FlowPIDController(float kp = 4.5f, float ki = 2.0f, float dt = 0.03f)
        : kp_(kp), ki_(ki), dt_(dt) {}

    void reset() {
        integral_ = 0.0f;
        lastOutputDuty_ = 0.0f;
    }

    // Output tracking bumpless transfer: initializes integral state to match active duty
    void transferState(float activeDuty, float feedforwardDuty, float error) {
        integral_ = activeDuty - feedforwardDuty - (kp_ * error);
    }

    float update(float targetFlow, float measuredFlow, float feedforwardDuty) {
        float error = targetFlow - measuredFlow;
        float pTerm = kp_ * error;

        float candidateIntegral = integral_ + ki_ * error * dt_;
        float candidateDuty = feedforwardDuty + pTerm + candidateIntegral;

        // Anti-windup conditional integration: clamp integral so duty is bounded
        if (candidateDuty > 100.0f && error > 0.0f) {
            integral_ = 100.0f - feedforwardDuty - pTerm;
        } else if (candidateDuty < 0.0f && error < 0.0f) {
            integral_ = 0.0f - feedforwardDuty - pTerm;
        } else {
            integral_ = candidateIntegral;
        }

        lastOutputDuty_ = std::clamp(feedforwardDuty + pTerm + integral_, 0.0f, 100.0f);
        return lastOutputDuty_;
    }

    float getIntegral() const { return integral_; }
    float getLastDuty() const { return lastOutputDuty_; }

private:
    float kp_;
    float ki_;
    float dt_;
    float integral_ = 0.0f;
    float lastOutputDuty_ = 0.0f;
};

// ============================================================================
// 6. Compound Recipe Engine (DNF Logic: OR of AND groups)
// ============================================================================
enum class ConditionMetric {
    PRESSURE,
    PUCK_FLOW,
    SCALE_FLOW,
    WEIGHT,
    VIRTUAL_VOLUME,
    PUMPED_WATER,
    PHASE_TIME
};

enum class ConditionOp {
    LTE,
    GTE,
    EQ
};

enum class PhaseExitReason : uint8_t {
    NONE = 0,
    TARGET_VOLUMETRIC = 1,
    TARGET_PRESSURE = 2,
    TARGET_FLOW = 3,
    TARGET_PUMPED = 4,
    DURATION = 5,
    SAFETY = 6,
    ABORTED = 7
};

struct TargetCondition {
    ConditionMetric metric;
    ConditionOp op;
    float value;

    bool evaluate(float metricValue) const {
        switch (op) {
        case ConditionOp::LTE: return metricValue <= value;
        case ConditionOp::GTE: return metricValue >= value;
        case ConditionOp::EQ:  return std::abs(metricValue - value) < 0.05f;
        }
        return false;
    }
};

struct ConditionGroup {
    std::vector<TargetCondition> andConditions; // All must be true (AND)
};

struct RecipeTelemetry {
    float pressure = 0.0f;
    float puckFlow = 0.0f;
    float scaleFlow = 0.0f;
    float weight = 0.0f;
    float virtualVolume = 0.0f;
    float pumpedWater = 0.0f;
    float phaseTime = 0.0f;
};

class CompoundRecipeEngine {
public:
    static PhaseExitReason evaluatePhase(const std::vector<ConditionGroup> &groups,
                                         float phaseDurationSec,
                                         const RecipeTelemetry &telem,
                                         float safetyLimitSec = 60.0f) {
        // 1. Check mandatory safety timeout
        if (telem.phaseTime >= safetyLimitSec) {
            return PhaseExitReason::SAFETY;
        }

        // 2. Evaluate DNF groups (OR across groups)
        for (const auto &group : groups) {
            if (group.andConditions.empty()) continue;

            bool allSatisfied = true;
            PhaseExitReason triggerReason = PhaseExitReason::NONE;

            for (const auto &cond : group.andConditions) {
                float val = 0.0f;
                switch (cond.metric) {
                case ConditionMetric::PRESSURE:
                    val = telem.pressure;
                    triggerReason = PhaseExitReason::TARGET_PRESSURE;
                    break;
                case ConditionMetric::PUCK_FLOW:
                case ConditionMetric::SCALE_FLOW:
                    val = (cond.metric == ConditionMetric::PUCK_FLOW) ? telem.puckFlow : telem.scaleFlow;
                    triggerReason = PhaseExitReason::TARGET_FLOW;
                    break;
                case ConditionMetric::WEIGHT:
                case ConditionMetric::VIRTUAL_VOLUME:
                    val = (cond.metric == ConditionMetric::WEIGHT) ? telem.weight : telem.virtualVolume;
                    triggerReason = PhaseExitReason::TARGET_VOLUMETRIC;
                    break;
                case ConditionMetric::PUMPED_WATER:
                    val = telem.pumpedWater;
                    triggerReason = PhaseExitReason::TARGET_PUMPED;
                    break;
                case ConditionMetric::PHASE_TIME:
                    val = telem.phaseTime;
                    triggerReason = PhaseExitReason::DURATION;
                    break;
                }

                if (!cond.evaluate(val)) {
                    allSatisfied = false;
                    break;
                }
            }

            if (allSatisfied) {
                return (triggerReason != PhaseExitReason::NONE) ? triggerReason : PhaseExitReason::TARGET_PRESSURE;
            }
        }

        // 3. Fallback to normal phase duration
        if (telem.phaseTime >= phaseDurationSec) {
            return PhaseExitReason::DURATION;
        }

        return PhaseExitReason::NONE;
    }
};

// ============================================================================
// 7. Software Pressure Ceiling & Mechanical Protections
// ============================================================================
class SoftwarePressureGuard {
public:
    static constexpr float kDefaultCeiling = 9.0f; // bar

    explicit SoftwarePressureGuard(float ceilingBar = kDefaultCeiling)
        : ceilingBar_(ceilingBar) {}

    float clampDuty(float currentPressure, float commandedDuty) const {
        if (currentPressure >= ceilingBar_ + 0.5f) {
            return 0.0f; // Hard electronic cutoff override
        }
        if (currentPressure > ceilingBar_) {
            // Proportional throttle to prevent OPV cracking
            float over = currentPressure - ceilingBar_;
            float throttle = std::max(0.0f, 1.0f - (over / 0.5f));
            return commandedDuty * throttle;
        }
        return commandedDuty;
    }

    float getCeiling() const { return ceilingBar_; }

private:
    float ceilingBar_;
};

// ============================================================================
// 8. Home Assistant MQTT Discovery & Telemetry Payload Generator
// ============================================================================
class HomeAssistantMqttHarness {
public:
    static std::string buildDiscoveryPayload(const std::string &mac) {
        std::string json = "{";
        json += "\"dev\":{\"ids\":\"" + mac + "\",\"name\":\"GaggiMate\",\"mf\":\"GaggiMate\",\"mdl\":\"Pro Headless\"},";
        json += "\"cmps\":{";
        json += "\"pressure\":{\"p\":\"sensor\",\"name\":\"Extraction Pressure\",\"unit_of_measurement\":\"bar\"},";
        json += "\"puck_flow\":{\"p\":\"sensor\",\"name\":\"Puck Flow Rate\",\"unit_of_measurement\":\"ml/s\"},";
        json += "\"scale_weight\":{\"p\":\"sensor\",\"name\":\"Beverage Weight\",\"unit_of_measurement\":\"g\"},";
        json += "\"thermal_fade\":{\"p\":\"sensor\",\"name\":\"Pump Thermal State\",\"unit_of_measurement\":\"%\"}";
        json += "}}";
        return json;
    }

    static std::string formatTelemetryPayload(float pressure, float puckFlow, float weight, float thermalFade) {
        // Sanitize NaN or Inf
        if (std::isnan(pressure) || std::isinf(pressure)) pressure = 0.0f;
        if (std::isnan(puckFlow) || std::isinf(puckFlow)) puckFlow = 0.0f;
        if (std::isnan(weight) || std::isinf(weight)) weight = 0.0f;
        if (std::isnan(thermalFade) || std::isinf(thermalFade)) thermalFade = 0.0f;

        char buf[128];
        std::snprintf(buf, sizeof(buf), "{\"pressure\":%.2f,\"puck_flow\":%.2f,\"weight\":%.1f,\"thermal_fade\":%.1f}",
                      pressure, puckFlow, weight, thermalFade * 100.0f);
        return std::string(buf);
    }
};

} // namespace gaggimate_test
