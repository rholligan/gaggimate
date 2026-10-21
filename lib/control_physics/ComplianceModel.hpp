#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace control_physics {

/**
 * @brief Dynamic Hydraulic Compliance Observer, Darcy Puck Resistance Estimator,
 *        and Channeling Auto-Save Guard for Gaggia Classic Pro (RI9480/14).
 *
 * Distinguishes liquid actually moving through the coffee bed (Q_puck) from water expanding
 * hoses and compressing air pockets (C_sys * dP/dt):
 *   Q_puck(t) = clamp(Q_raw(t) - C_sys(P) * (dP/dt), 0.0, Q_max)
 *   C_sys(P) = V_air,0 / (P + 1.0)^2 + c_hose
 *
 * Evaluates instantaneous Darcy resistance R_puck = P / (Q_puck^2 + eps).
 * Once the coffee bed has reached steady-state extraction, if d(ln R_puck)/dt drops faster
 * than -0.18 s^-1, an active channel blowout is detected and an Auto-Save flow limit (1.8 ml/s) is triggered.
 */
class ComplianceModel {
  public:
    static constexpr float kDefaultVair0 = 6.0f;          // Initial uncompressed air pocket volume (ml)
    static constexpr float kDefaultChose = 0.15f;         // High-pressure line elasticity (ml/bar)
    static constexpr float kDarcyEps = 1e-4f;             // Darcy divisor regularizer (ml^2/s^2)
    static constexpr float kChannelingThreshold = -0.18f; // d(ln R)/dt threshold for channel blowout (1/s)
    static constexpr float kAutoSaveFlowTarget = 1.8f;    // Protective capped flow setpoint (ml/s)

    ComplianceModel(float dt = 0.02f) : dt_(dt) {
        reset();
    }

    void reset() noexcept {
        P_prev_ = 0.0f;
        dEdt_filtered_ = 0.0f;
        Rpuck_ = 4.0f;
        Rpuck_prev_ = 4.0f;
        dlnR_dt_ = 0.0f;
        totalPumpVolume_ = 0.0f;
        totalPuckVolume_ = 0.0f;
        autoSaveActive_ = false;
        initialized_ = false;
        settleTimer_ = 0.0f;
        settled_ = false;
    }

    void setComplianceParams(float Vair0, float Chose) noexcept {
        Vair0_ = std::max(0.0f, Vair0);
        Chose_ = std::max(0.0f, Chose);
    }

    /**
     * @brief Computes non-linear hydraulic compliance C_sys(P) in ml/bar at pressure P.
     */
    [[nodiscard]] float getSystemCompliance(float pressureBar) const noexcept {
        const float P = std::max(0.0f, pressureBar);
        const float denom = P + 1.0f;
        return (Vair0_ / (denom * denom)) + Chose_;
    }

    /**
     * @brief Executes one 50Hz update cycle.
     */
    float update(float pressureBar, float rawPumpFlowMlS) noexcept {
        const float P = std::max(0.0f, pressureBar);
        const float Q_raw = std::max(0.0f, rawPumpFlowMlS);

        if (!initialized_) {
            P_prev_ = P;
            initialized_ = true;
            return 0.0f;
        }

        // 1. Filtered pressure derivative (low-pass smoothing @ ~3Hz)
        const float raw_dP_dt = (P - P_prev_) / dt_;
        P_prev_ = P;
        constexpr float alpha = 0.28f;
        dEdt_filtered_ = alpha * raw_dP_dt + (1.0f - alpha) * dEdt_filtered_;

        // 2. Compliance flux and true puck flow
        const float C_sys = getSystemCompliance(P);
        const float complianceFlux = C_sys * dEdt_filtered_;
        const float Q_puck = std::clamp(Q_raw - complianceFlux, 0.0f, 15.0f);

        // Volumetric integration
        totalPumpVolume_ += Q_raw * dt_;
        totalPuckVolume_ += Q_puck * dt_;

        // 3. Real-time Darcy Puck Resistance: R_puck = P / (Q_puck^2 + eps) [bar*s^2 / ml^2]
        if (P > 1.5f && Q_puck > 0.4f) {
            const float new_Rpuck = P / (Q_puck * Q_puck + kDarcyEps);
            Rpuck_ = 0.15f * new_Rpuck + 0.85f * Rpuck_;

            if (Rpuck_ > 1e-3f && Rpuck_prev_ > 1e-3f) {
                const float instant_dlnR = (std::log(Rpuck_) - std::log(Rpuck_prev_)) / dt_;
                dlnR_dt_ = 0.10f * instant_dlnR + 0.90f * dlnR_dt_;
            }
            Rpuck_prev_ = Rpuck_;

            settleTimer_ += dt_;
            // Bed settles after 2.0s of sustained pressure once dlnR stabilizes within [-0.05, 0.05]
            if (settleTimer_ > 2.0f && std::abs(dlnR_dt_) < 0.05f) {
                settled_ = true;
            }

            // 4. Auto-Save Channeling Guard:
            // Only triggers once bed has settled and d(ln R)/dt drops below -0.18 s^-1
            if (settled_ && P > 4.0f && Q_puck > 0.8f && dlnR_dt_ < kChannelingThreshold) {
                autoSaveActive_ = true;
            }
        } else {
            settleTimer_ = 0.0f;
            settled_ = false;
        }

        return Q_puck;
    }

    [[nodiscard]] float getPuckResistance() const noexcept { return Rpuck_; }
    [[nodiscard]] float getResistanceLogDerivative() const noexcept { return dlnR_dt_; }
    [[nodiscard]] float getTotalPuckVolume() const noexcept { return totalPuckVolume_; }
    [[nodiscard]] float getTotalPumpVolume() const noexcept { return totalPumpVolume_; }
    [[nodiscard]] bool isAutoSaveActive() const noexcept { return autoSaveActive_; }
    [[nodiscard]] float getPressureDerivative() const noexcept { return dEdt_filtered_; }

  private:
    float dt_;
    float Vair0_{kDefaultVair0};
    float Chose_{kDefaultChose};

    float P_prev_{0.0f};
    float dEdt_filtered_{0.0f};
    float Rpuck_{4.0f};
    float Rpuck_prev_{4.0f};
    float dlnR_dt_{0.0f};

    float totalPumpVolume_{0.0f};
    float totalPuckVolume_{0.0f};
    float settleTimer_{0.0f};
    bool settled_{false};
    bool autoSaveActive_{false};
    bool initialized_{false};
};

} // namespace control_physics
