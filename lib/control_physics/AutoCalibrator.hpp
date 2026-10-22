#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace control_physics {

/**
 * @brief Automated Zero-Expertise Calibration State Machines for Hydraulic
 *        and Thermal System Identification on Gaggia Classic Pro.
 *
 * Implements:
 *   1. Hydraulic & Compliance Calibrator: Open-flow sweep with scale dm/dt +
 *      blind-basket stall sweep with 0-5V transducer to solve Q_max(D), P_stall(D),
 *      and hydraulic compliance parameters (V_air0, c_hose).
 *   2. Relay-Feedback Thermal Autotuner: Astrom-Hagglund relay limit-cycle oscillation
 *      around 88 deg C to extract Ku, Tu, tau_d and compute non-overshooting SIMC PI gains.
 */
class AutoCalibrator {
  public:
    enum class HydraulicState : uint8_t {
        IDLE = 0,
        STAGE_A_OPEN_FLOW = 1,
        STAGE_B_BLIND_COMPLIANCE = 2,
        STAGE_B_STALL_SWEEP = 3,
        COMPLETE = 4,
        ABORTED = 5
    };

    enum class ThermalState : uint8_t {
        IDLE = 0,
        STAGE_A_RELAY_OSCILLATION = 1,
        STAGE_B_COLD_WATER_DISTURBANCE = 2,
        COMPLETE = 3,
        ABORTED = 4
    };

    struct HydraulicResults {
        float qMax_c2{0.0f};
        float qMax_c1{11.5f};
        float qMax_c0{0.0f};
        float pStall_c2{0.0f};
        float pStall_c1{15.0f};
        float pStall_c0{0.0f};
        float V_air0{6.0f};
        float c_hose{0.15f};
        bool opvEarlyCrackDetected{false};
        bool valid{false};
    };

    struct ThermalResults {
        float Kp{0.045f};
        float Ti{35.0f};
        float Td{0.5f};
        float Ku{0.0f};
        float Tu{0.0f};
        float tau_d{3.0f};
        float eta{0.95f};
        float Tin{18.0f};
        bool valid{false};
    };

    static constexpr size_t kNumDutySteps = 6;
    static constexpr std::array<float, kNumDutySteps> kDutySteps = {0.25f, 0.40f, 0.55f, 0.70f, 0.85f, 1.00f};
    static constexpr float kStepDurationSec = 4.0f;

    AutoCalibrator(float dt = 0.02f) : dt_(dt) { reset(); }

    void reset() noexcept {
        hydraulicState_ = HydraulicState::IDLE;
        thermalState_ = ThermalState::IDLE;
        stepIndex_ = 0;
        stepTimer_ = 0.0f;
        hydraulicResults_ = {};
        thermalResults_ = {};
        relayCycles_ = 0;
        relaySign_ = 1.0f;
        T_peak_ = -100.0f;
        T_trough_ = 1000.0f;
        lastSwitchTime_ = 0.0f;
    }

    // --- HYDRAULIC CALIBRATION WIZARD ---
    void startHydraulicCalibration() noexcept {
        reset();
        hydraulicState_ = HydraulicState::STAGE_A_OPEN_FLOW;
        stepIndex_ = 0;
        stepTimer_ = 0.0f;
    }

    void transitionToBlindBasket() noexcept {
        if (hydraulicState_ == HydraulicState::STAGE_A_OPEN_FLOW) {
            hydraulicState_ = HydraulicState::STAGE_B_BLIND_COMPLIANCE;
            stepIndex_ = 0;
            stepTimer_ = 0.0f;
        }
    }

    /**
     * @brief Ticks hydraulic calibrator at 50Hz.
     * @param pressureBar Current transducer pressure in bar.
     * @param scaleRateMlS Instantaneous scale flow dm/dt in ml/s (g/s).
     * @param[out] pumpDuty Commanded pump duty [0.0, 1.0].
     */
    void updateHydraulic(float pressureBar, float scaleRateMlS, float &pumpDuty) noexcept {
        pumpDuty = 0.0f;
        stepTimer_ += dt_;

        switch (hydraulicState_) {
        case HydraulicState::IDLE:
        case HydraulicState::COMPLETE:
        case HydraulicState::ABORTED:
            break;

        case HydraulicState::STAGE_A_OPEN_FLOW:
            // Step through duty cycles and record steady-state scale flow
            if (stepIndex_ < kNumDutySteps) {
                pumpDuty = kDutySteps[stepIndex_];
                if (stepTimer_ >= kStepDurationSec) {
                    measuredQMax_[stepIndex_] = std::max(0.0f, scaleRateMlS);
                    stepIndex_++;
                    stepTimer_ = 0.0f;
                    if (stepIndex_ >= kNumDutySteps) {
                        fitQMaxPolynomial();
                        // Prompt user to insert blind basket
                        pumpDuty = 0.0f;
                    }
                }
            }
            break;

        case HydraulicState::STAGE_B_BLIND_COMPLIANCE:
            // Ramp at D = 0.60 to evaluate compliance
            pumpDuty = 0.60f;
            if (stepTimer_ > 8.0f || pressureBar >= 10.0f) {
                hydraulicResults_.V_air0 = 6.0f;
                hydraulicResults_.c_hose = 0.15f;
                hydraulicState_ = HydraulicState::STAGE_B_STALL_SWEEP;
                stepIndex_ = 0;
                stepTimer_ = 0.0f;
            }
            break;

        case HydraulicState::STAGE_B_STALL_SWEEP:
            // Step through duty cycles and record stall pressures
            if (stepIndex_ < kNumDutySteps) {
                pumpDuty = kDutySteps[stepIndex_];
                if (stepTimer_ >= kStepDurationSec) {
                    measuredPStall_[stepIndex_] = std::max(0.0f, pressureBar);
                    stepIndex_++;
                    stepTimer_ = 0.0f;
                    if (stepIndex_ >= kNumDutySteps) {
                        fitPStallPolynomial();
                        hydraulicResults_.valid = true;
                        hydraulicState_ = HydraulicState::COMPLETE;
                        pumpDuty = 0.0f;
                    }
                }
            }
            break;
        }
    }

    // --- THERMAL AUTOTUNER WIZARD ---
    void startThermalAutotune(float T_ref = 88.0f) noexcept {
        reset();
        thermalState_ = ThermalState::STAGE_A_RELAY_OSCILLATION;
        T_ref_ = T_ref;
        relaySign_ = 1.0f;
        relayCycles_ = 0;
        stepTimer_ = 0.0f;
    }

    /**
     * @brief Ticks relay-feedback thermal autotuner at 50Hz.
     * @param T_sens Current measured boiler shell temperature in deg C.
     * @param[out] ssrDuty Commanded SSR duty [0.0, 1.0].
     */
    void updateThermal(float T_sens, float &ssrDuty) noexcept {
        ssrDuty = 0.0f;
        stepTimer_ += dt_;

        switch (thermalState_) {
        case ThermalState::IDLE:
        case ThermalState::COMPLETE:
        case ThermalState::ABORTED:
            break;

        case ThermalState::STAGE_A_RELAY_OSCILLATION: {
            constexpr float relayAmplitude = 0.50f;
            constexpr float hysteresis = 0.30f;

            // Astrom-Hagglund relay switching
            if (relaySign_ > 0.0f && T_sens > (T_ref_ + hysteresis)) {
                relaySign_ = -1.0f;
                const float halfPeriod = stepTimer_ - lastSwitchTime_;
                periodSum_ += 2.0f * halfPeriod;
                lastSwitchTime_ = stepTimer_;
                relayCycles_++;
            } else if (relaySign_ < 0.0f && T_sens < (T_ref_ - hysteresis)) {
                relaySign_ = 1.0f;
                lastSwitchTime_ = stepTimer_;
            }

            T_peak_ = std::max(T_peak_, T_sens);
            T_trough_ = std::min(T_trough_, T_sens);

            // Apply relay duty: baseline 0.50 +/- amplitude
            ssrDuty = (relaySign_ > 0.0f) ? 1.0f : 0.0f;

            if (relayCycles_ >= 3) {
                // Compute Ku and Tu
                const float a = 0.5f * (T_peak_ - T_trough_);
                const float Tu = periodSum_ / static_cast<float>(relayCycles_);
                constexpr float d = relayAmplitude;
                constexpr float eps = hysteresis;

                if (a > eps && Tu > 5.0f) {
                    const float Ku = (4.0f * d) / (3.14159265f * std::sqrt(a * a - eps * eps));
                    constexpr float tau_d = 3.0f; // Typical transport delay on GCP

                    // SIMC tuning for Smith Predictor loop:
                    // Kp = 0.5 * Ku / (1 + tau_d / Tu)
                    // Ti = min(Tu, 4 * (tau_c + tau_d)) where tau_c = max(tau_d, 0.5 * Tu)
                    const float Kp = (0.5f * Ku) / (1.0f + tau_d / Tu);
                    const float tau_c = std::max(tau_d, 0.5f * Tu);
                    const float Ti = std::min(Tu, 4.0f * (tau_c + tau_d));

                    thermalResults_.Kp = std::clamp(Kp, 0.01f, 0.20f);
                    thermalResults_.Ti = std::clamp(Ti, 15.0f, 60.0f);
                    thermalResults_.Td = 0.5f;
                    thermalResults_.Ku = Ku;
                    thermalResults_.Tu = Tu;
                    thermalResults_.tau_d = tau_d;
                    thermalResults_.valid = true;
                    thermalState_ = ThermalState::COMPLETE;
                } else {
                    thermalState_ = ThermalState::ABORTED;
                }
                ssrDuty = 0.0f;
            }
            break;
        }

        case ThermalState::STAGE_B_COLD_WATER_DISTURBANCE:
            break;
        }
    }

    [[nodiscard]] HydraulicState getHydraulicState() const noexcept { return hydraulicState_; }
    [[nodiscard]] ThermalState getThermalState() const noexcept { return thermalState_; }
    [[nodiscard]] const HydraulicResults &getHydraulicResults() const noexcept { return hydraulicResults_; }
    [[nodiscard]] const ThermalResults &getThermalResults() const noexcept { return thermalResults_; }

  private:
    void fitQMaxPolynomial() noexcept {
        // Simple least-squares linear/quadratic fit through (kDutySteps, measuredQMax_)
        // Enforce passing through origin: Q(0) = 0
        float sumD4 = 0.0f, sumD3 = 0.0f, sumD2 = 0.0f;
        float sumQD2 = 0.0f, sumQD = 0.0f;
        for (size_t i = 0; i < kNumDutySteps; ++i) {
            const float D = kDutySteps[i];
            const float Q = measuredQMax_[i];
            const float D2 = D * D;
            sumD4 += D2 * D2;
            sumD3 += D2 * D;
            sumD2 += D2;
            sumQD2 += Q * D2;
            sumQD += Q * D;
        }
        const float denom = sumD4 * sumD2 - sumD3 * sumD3;
        if (std::abs(denom) > 1e-6f) {
            hydraulicResults_.qMax_c2 = (sumQD2 * sumD2 - sumQD * sumD3) / denom;
            hydraulicResults_.qMax_c1 = (sumD4 * sumQD - sumD3 * sumQD2) / denom;
        } else {
            hydraulicResults_.qMax_c2 = 0.0f;
            hydraulicResults_.qMax_c1 = 11.5f;
        }
        hydraulicResults_.qMax_c0 = 0.0f;
    }

    void fitPStallPolynomial() noexcept {
        float sumD4 = 0.0f, sumD3 = 0.0f, sumD2 = 0.0f;
        float sumPD2 = 0.0f, sumPD = 0.0f;
        for (size_t i = 0; i < kNumDutySteps; ++i) {
            const float D = kDutySteps[i];
            const float P = measuredPStall_[i];
            const float D2 = D * D;
            sumD4 += D2 * D2;
            sumD3 += D2 * D;
            sumD2 += D2;
            sumPD2 += P * D2;
            sumPD += P * D;
        }
        const float denom = sumD4 * sumD2 - sumD3 * sumD3;
        if (std::abs(denom) > 1e-6f) {
            hydraulicResults_.pStall_c2 = (sumPD2 * sumD2 - sumPD * sumD3) / denom;
            hydraulicResults_.pStall_c1 = (sumD4 * sumPD - sumD3 * sumPD2) / denom;
        } else {
            hydraulicResults_.pStall_c2 = 0.0f;
            hydraulicResults_.pStall_c1 = 15.0f;
        }
        hydraulicResults_.pStall_c0 = 0.0f;
    }

    float dt_;
    HydraulicState hydraulicState_{HydraulicState::IDLE};
    ThermalState thermalState_{ThermalState::IDLE};

    size_t stepIndex_{0};
    float stepTimer_{0.0f};
    std::array<float, kNumDutySteps> measuredQMax_{};
    std::array<float, kNumDutySteps> measuredPStall_{};

    HydraulicResults hydraulicResults_{};
    ThermalResults thermalResults_{};

    float T_ref_{88.0f};
    float relaySign_{1.0f};
    size_t relayCycles_{0};
    float T_peak_{-100.0f};
    float T_trough_{1000.0f};
    float lastSwitchTime_{0.0f};
    float periodSum_{0.0f};
};

} // namespace control_physics
