#ifndef PUMP_SURFACE_MODEL_H
#define PUMP_SURFACE_MODEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>

// Struct representing the 5-point nodal vectors for the linear displacement pump surface.
struct PumpCalibrationData {
    static constexpr size_t NUM_POINTS = 5;
    float dutyPoints[NUM_POINTS] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
    float qMaxPoints[NUM_POINTS] = {1.8f, 3.5f, 5.5f, 7.2f, 8.5f};     // ml/s at P=0
    float pStallPoints[NUM_POINTS] = {4.0f, 6.5f, 9.5f, 12.0f, 14.0f}; // bar at Q=0
    float systemCompliance = 1.4f;                                     // ml/bar
    float thermalFadeFactor = 0.15f;                                   // kappa_fade

    bool isValid() const {
        if (std::isnan(systemCompliance) || std::isnan(thermalFadeFactor) || std::isinf(systemCompliance) ||
            std::isinf(thermalFadeFactor)) {
            return false;
        }
        if (systemCompliance < 0.2f || systemCompliance > 5.0f) {
            return false;
        }
        if (thermalFadeFactor < 0.0f || thermalFadeFactor > 0.5f) {
            return false;
        }
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            if (std::isnan(dutyPoints[i]) || std::isnan(qMaxPoints[i]) || std::isnan(pStallPoints[i]) ||
                std::isinf(dutyPoints[i]) || std::isinf(qMaxPoints[i]) || std::isinf(pStallPoints[i])) {
                return false;
            }
            if (dutyPoints[i] < 0.0f || dutyPoints[i] > 100.0f) {
                return false;
            }
            if (qMaxPoints[i] <= 0.0f || pStallPoints[i] <= 0.0f) {
                return false;
            }
            if (i > 0) {
                if (dutyPoints[i] <= dutyPoints[i - 1]) {
                    return false;
                }
                if (qMaxPoints[i] <= qMaxPoints[i - 1]) {
                    return false;
                }
                if (pStallPoints[i] <= pStallPoints[i - 1]) {
                    return false;
                }
            }
        }
        if (pStallPoints[NUM_POINTS - 1] < 6.0f || pStallPoints[NUM_POINTS - 1] > 20.0f) {
            return false;
        }
        if (qMaxPoints[NUM_POINTS - 1] < 3.0f || qMaxPoints[NUM_POINTS - 1] > 25.0f) {
            return false;
        }
        return true;
    }

    bool operator==(const PumpCalibrationData &other) const {
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            if (std::fabs(dutyPoints[i] - other.dutyPoints[i]) > 1e-4f) {
                return false;
            }
            if (std::fabs(qMaxPoints[i] - other.qMaxPoints[i]) > 1e-4f) {
                return false;
            }
            if (std::fabs(pStallPoints[i] - other.pStallPoints[i]) > 1e-4f) {
                return false;
            }
        }
        return std::fabs(systemCompliance - other.systemCompliance) <= 1e-4f &&
               std::fabs(thermalFadeFactor - other.thermalFadeFactor) <= 1e-4f;
    }

    bool operator!=(const PumpCalibrationData &other) const { return !(*this == other); }
};

// Physical 2D displacement surface model for reciprocating vibration pump (Ulka EX5).
// Governing equation: Q(P, Duty) = Qmax(Duty) * (1 - P / Pstall(Duty))
class PumpSurfaceModel {
  public:
    static constexpr float DEFAULT_DUTY_DEADBAND = 15.0f; // Minimum phase-cut conduction percentage

    PumpSurfaceModel() : _cal(), _dutyDeadband(DEFAULT_DUTY_DEADBAND) {}

    explicit PumpSurfaceModel(const PumpCalibrationData &cal, float dutyDeadband = DEFAULT_DUTY_DEADBAND)
        : _cal(cal.isValid() ? cal : PumpCalibrationData()), _dutyDeadband(std::clamp(dutyDeadband, 0.0f, 100.0f)) {}

    void setCalibration(const PumpCalibrationData &cal) {
        if (cal.isValid()) {
            _cal = cal;
        }
    }

    const PumpCalibrationData &getCalibration() const { return _cal; }

    void resetToFactoryDefaults() {
        _cal = PumpCalibrationData();
        _dutyDeadband = DEFAULT_DUTY_DEADBAND;
    }

    void setDutyDeadband(float deadbandPercent) { _dutyDeadband = std::clamp(deadbandPercent, 0.0f, 100.0f); }
    float getDutyDeadband() const { return _dutyDeadband; }

    // Evaluates open-circuit flow Qmax(Duty) in ml/s with piecewise linear interpolation across duty nodes.
    float getMaxFlow(float dutyPercent, float thermalScalar = 1.0f) const {
        if (std::isnan(dutyPercent) || std::isnan(thermalScalar) || thermalScalar <= 0.0f) {
            return 0.0f;
        }
        const float raw = interpolateNodalVector(_cal.qMaxPoints, dutyPercent);
        const float scalar = std::clamp(thermalScalar, 0.0f, 1.0f);
        return std::max(0.0f, raw * scalar);
    }

    // Evaluates blind stall pressure Pstall(Duty) in bar with piecewise linear interpolation across duty nodes.
    float getStallPressure(float dutyPercent, float thermalScalar = 1.0f) const {
        if (std::isnan(dutyPercent) || std::isnan(thermalScalar) || thermalScalar <= 0.0f) {
            return 0.0f;
        }
        const float raw = interpolateNodalVector(_cal.pStallPoints, dutyPercent);
        const float scalar = std::clamp(thermalScalar, 0.0f, 1.0f);
        return std::max(0.0f, raw * scalar);
    }

    // Evaluates delivered pump flow Q(P, Duty) in ml/s with boundary clamping and physical invariants.
    float getFlow(float pressureBar, float dutyPercent, float thermalScalar = 1.0f) const {
        if (std::isnan(pressureBar) || std::isnan(dutyPercent) || std::isnan(thermalScalar) || dutyPercent <= _dutyDeadband ||
            thermalScalar <= 0.0f) {
            return 0.0f;
        }
        const float qMax = getMaxFlow(dutyPercent, thermalScalar);
        const float pStall = getStallPressure(dutyPercent, thermalScalar);
        if (qMax <= 1e-4f || pStall <= 1e-4f) {
            return 0.0f;
        }
        // Physical stall boundary clamp
        if (pressureBar >= pStall) {
            return 0.0f;
        }
        // Clamped for negative gauge pressure
        if (pressureBar <= 0.0f) {
            return qMax;
        }
        // Linear displacement throttling equation
        const float flow = qMax * (1.0f - (pressureBar / pStall));
        if (std::isnan(flow) || std::isinf(flow)) {
            return 0.0f;
        }
        return std::clamp(flow, 0.0f, qMax);
    }

    // Inverted monotonic root-finding: determines required feedforward duty percentage for target flow and pressure.
    float getFeedforwardDuty(float targetFlowMlS, float pressureBar, float thermalScalar = 1.0f) const {
        if (std::isnan(targetFlowMlS) || std::isnan(pressureBar) || std::isnan(thermalScalar)) {
            return 0.0f;
        }
        if (targetFlowMlS <= 0.0f || thermalScalar <= 0.0f || std::isinf(targetFlowMlS) || std::isinf(thermalScalar) ||
            (std::isinf(pressureBar) && pressureBar > 0.0f)) {
            return 0.0f;
        }
        if (pressureBar < 0.0f) {
            pressureBar = 0.0f;
        }
        const float qMaxAtFull = getFlow(pressureBar, 100.0f, thermalScalar);
        if (qMaxAtFull <= 1e-4f) {
            // Target is unattainable against this pressure even at 100% duty
            return 100.0f;
        }
        if (targetFlowMlS >= qMaxAtFull) {
            return 100.0f;
        }

        // Monotonic bounded bisection search across [deadband, 100.0%]
        float low = _dutyDeadband;
        float high = 100.0f;
        for (int iter = 0; iter < 24; ++iter) {
            const float mid = 0.5f * (low + high);
            const float qMid = getFlow(pressureBar, mid, thermalScalar);
            if (qMid < targetFlowMlS) {
                low = mid;
            } else {
                high = mid;
            }
        }
        const float bestDuty = 0.5f * (low + high);
        return std::clamp(bestDuty, 0.0f, 100.0f);
    }

  private:
    PumpCalibrationData _cal;
    float _dutyDeadband = DEFAULT_DUTY_DEADBAND;

    float interpolateNodalVector(const float *yVals, float duty) const {
        if (std::isnan(duty) || duty <= _dutyDeadband) {
            return 0.0f;
        }
        // Sub-node linear interpolation between deadband and first calibrated point
        if (duty <= _cal.dutyPoints[0]) {
            const float span = _cal.dutyPoints[0] - _dutyDeadband;
            if (span <= 1e-4f) {
                return yVals[0];
            }
            const float alpha = (duty - _dutyDeadband) / span;
            return std::clamp(alpha * yVals[0], 0.0f, yVals[0]);
        }
        // Beyond upper node clamp
        if (duty >= _cal.dutyPoints[PumpCalibrationData::NUM_POINTS - 1]) {
            return yVals[PumpCalibrationData::NUM_POINTS - 1];
        }
        // Piecewise linear interpolation between nodes
        for (size_t i = 0; i < PumpCalibrationData::NUM_POINTS - 1; ++i) {
            if (duty >= _cal.dutyPoints[i] && duty <= _cal.dutyPoints[i + 1]) {
                const float span = _cal.dutyPoints[i + 1] - _cal.dutyPoints[i];
                if (span <= 1e-4f) {
                    return yVals[i];
                }
                const float alpha = (duty - _cal.dutyPoints[i]) / span;
                return (1.0f - alpha) * yVals[i] + alpha * yVals[i + 1];
            }
        }
        return 0.0f;
    }
};

#endif // PUMP_SURFACE_MODEL_H
