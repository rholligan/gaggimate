#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace control_physics {

/**
 * @brief Convex Non-Linear Ulka Vibratory Pump Model with Solenoid Coil Thermal Fade
 *        and Recursive Least Squares (RLS) Adaptation.
 *
 * Models reciprocating solenoid pump dynamics (50Hz mains) under phase-angle dimming:
 *   Q_raw(P, D, E_coil) = alpha_coil(E_coil) * Q_max(D) * [1 - (min(P, P_stall'(D)) / P_stall'(D))^kappa(D)]
 * Accounts for coil thermal degradation (10-15% drop over extended shots) and inverts
 * the surface numerically to determine exact feed-forward duty for target flow rates.
 */
class PumpSurface {
  public:
    struct Polynomial3 {
        float c2{0.0f};
        float c1{0.0f};
        float c0{0.0f};

        [[nodiscard]] float eval(float x) const noexcept {
            return c2 * x * x + c1 * x + c0;
        }
    };

    static constexpr float kDefaultTauCool = 95.0f;     // Coil thermal cooling time constant (s)
    static constexpr float kDefaultBetaFade = 0.0028f;   // Coil thermal fade coefficient (1/s)
    static constexpr float kMinAlphaCoil = 0.60f;       // Minimum coil degradation floor
    static constexpr float kDefaultKappa = 1.45f;       // Empirical convexity exponent

    PumpSurface(float dt = 0.02f) : dt_(dt) {
        resetCoil();
        setDefaultCalibration();
    }

    void resetCoil() noexcept {
        E_coil_ = 0.0f;
    }

    void setDefaultCalibration() noexcept {
        // Factory baseline for 50Hz Ulka EX5 on 230-240V mains:
        // Q_max(D) = a1*D^2 + a2*D + a3 (ml/s at P = 0 bar)
        // D=1.0 -> ~11.5 ml/s; D=0.6 -> ~7.2 ml/s; D=0.25 -> ~2.4 ml/s; D=0 -> 0 ml/s
        qMaxPoly_ = {-1.5f, 13.0f, 0.0f};

        // P_stall(D) = b1*D^2 + b2*D + b3 (bar at Q = 0 ml/s)
        // D=1.0 -> ~15.5 bar; D=0.6 -> ~9.5 bar; D=0.3 -> ~3.5 bar; D=0 -> 0 bar
        pStallPoly_ = {-2.0f, 17.5f, 0.0f};

        kappa_ = kDefaultKappa;
    }

    void setQMaxPoly(float c2, float c1, float c0) noexcept {
        qMaxPoly_ = {c2, c1, c0};
    }

    void setPStallPoly(float c2, float c1, float c0) noexcept {
        pStallPoly_ = {c2, c1, c0};
    }

    void setConvexity(float kappa) noexcept {
        kappa_ = std::clamp(kappa, 1.1f, 2.0f);
    }

    /**
     * @brief Updates solenoid coil thermal energy accumulator:
     *        dE_coil/dt = D^2 - E_coil / tau_cool
     */
    void updateThermalCoil(float duty) noexcept {
        const float D = std::clamp(duty, 0.0f, 1.0f);
        const float dE = (D * D - E_coil_ / kDefaultTauCool) * dt_;
        E_coil_ = std::max(0.0f, E_coil_ + dE);
    }

    /**
     * @brief Current coil thermal efficiency factor alpha_coil in [0.60, 1.00].
     */
    [[nodiscard]] float getCoilAlpha() const noexcept {
        return std::clamp(1.0f - kDefaultBetaFade * E_coil_, kMinAlphaCoil, 1.0f);
    }

    /**
     * @brief Evaluates open-circuit flow Q_max(D) in ml/s at P = 0 bar.
     */
    [[nodiscard]] float getQMax(float duty) const noexcept {
        if (duty <= 0.01f) return 0.0f;
        const float D = std::clamp(duty, 0.0f, 1.0f);
        return std::max(0.0f, qMaxPoly_.eval(D));
    }

    /**
     * @brief Evaluates deadhead stall pressure P_stall(D) in bar at Q = 0 ml/s,
     *        scaled by current coil thermal degradation.
     */
    [[nodiscard]] float getPStall(float duty) const noexcept {
        if (duty <= 0.01f) return 0.0f;
        const float D = std::clamp(duty, 0.0f, 1.0f);
        const float unadjustedPstall = std::max(0.0f, pStallPoly_.eval(D));
        return getCoilAlpha() * unadjustedPstall;
    }

    /**
     * @brief Predicts raw pump flow Q_raw(P, D) in ml/s for given pressure and duty.
     *
     * @param pressureBar Current transducer pressure in bar.
     * @param duty Pump phase-cut duty ratio in [0.0, 1.0].
     * @return Estimated pump flow rate in ml/s.
     */
    [[nodiscard]] float predictFlow(float pressureBar, float duty) const noexcept {
        if (duty <= 0.01f || pressureBar < 0.0f) {
            return 0.0f;
        }
        const float P = std::max(0.0f, pressureBar);
        const float D = std::clamp(duty, 0.0f, 1.0f);
        const float alpha = getCoilAlpha();
        const float qMax = alpha * getQMax(D);
        const float pStall = getPStall(D);

        if (pStall <= 0.01f || qMax <= 0.001f) {
            return 0.0f;
        }

        if (P >= pStall) {
            return 0.0f; // Stalled: pressure matches or exceeds pump driving capability
        }

        const float pressureRatio = std::clamp(P / pStall, 0.0f, 1.0f);
        const float flow = qMax * (1.0f - std::pow(pressureRatio, kappa_));
        return std::max(0.0f, flow);
    }

    /**
     * @brief Inverts the non-linear pump surface to find the required dimmer duty
     *        to deliver targetFlowMlS against the specified pressure.
     *
     * Uses monotonic bounded bisection (10 iterations -> error < 0.001 duty).
     *
     * @param targetFlowMlS Desired flow rate in ml/s.
     * @param pressureBar Current hydraulic pressure in bar.
     * @return Commanded duty in [0.0, 1.0].
     */
    [[nodiscard]] float invertDuty(float targetFlowMlS, float pressureBar) const noexcept {
        if (targetFlowMlS <= 0.01f) {
            return 0.0f;
        }

        float low = 0.0f;
        float high = 1.0f;

        // Check if max duty (1.0) can even reach the target
        const float flowAtMax = predictFlow(pressureBar, 1.0f);
        if (flowAtMax <= targetFlowMlS) {
            return 1.0f;
        }

        // Bisection search
        for (int i = 0; i < 10; ++i) {
            const float mid = 0.5f * (low + high);
            const float flowMid = predictFlow(pressureBar, mid);
            if (flowMid < targetFlowMlS) {
                low = mid;
            } else {
                high = mid;
            }
        }

        return std::clamp(0.5f * (low + high), 0.0f, 1.0f);
    }

    /**
     * @brief Online Recursive Least Squares (RLS) fine-tuner:
     *        Adjusts convexity exponent kappa based on ground-truth scale flow
     *        during steady extraction (dP/dt ~ 0).
     */
    void updateRLS(float estimatedQ, float scaleFlowRate, float lambda = 0.05f) noexcept {
        if (scaleFlowRate <= 0.2f || estimatedQ <= 0.2f) {
            return;
        }
        const float errorRatio = scaleFlowRate / estimatedQ;
        // If scale flow is higher than predicted at high pressure, surface is more convex (higher kappa)
        if (std::isfinite(errorRatio) && errorRatio > 0.5f && errorRatio < 2.0f) {
            kappa_ += lambda * (errorRatio - 1.0f) * 0.2f;
            kappa_ = std::clamp(kappa_, 1.15f, 1.85f);
        }
    }

    [[nodiscard]] float getConvexity() const noexcept { return kappa_; }
    [[nodiscard]] float getCoilEnergy() const noexcept { return E_coil_; }

  private:
    float dt_;
    Polynomial3 qMaxPoly_{};
    Polynomial3 pStallPoly_{};
    float kappa_{kDefaultKappa};
    float E_coil_{0.0f};
};

} // namespace control_physics
