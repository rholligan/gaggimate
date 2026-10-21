#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace control_physics {

/**
 * @brief 2-Node Lumped-Capacitance Thermal State Observer, Smith Predictor,
 *        and Zero-Lag Flow-Coupled Feed-Forward Controller for Gaggia Classic Pro (RI9480/14).
 *
 * Implements discrete-time state-space energy balances at 50Hz (dt = 0.02s) to track internal
 * boiler water temperature T_b and brass group head temperature T_g, compensating for the 3s
 * transport dead-time of the exterior shell sensor and eliminating cold-water brew droop.
 */
class ThermalObserver {
  public:
    // Physical thermal constants for RI9480/14 dry-cast aluminum boiler + brass group
    static constexpr float kDefaultCb = 795.1f;       // Effective boiler thermal capacitance (J/K) [Cw ~418.4, C_Al ~376.7]
    static constexpr float kDefaultCg = 437.0f;       // Brass group head thermal capacitance (J/K) [1.15 kg * 380 J/kg*K]
    static constexpr float kDefaultGbg = 2.8f;        // Flange conductive conductance boiler -> group (W/K)
    static constexpr float kDefaultGbAmb = 0.9f;      // Convective loss boiler -> ambient (W/K)
    static constexpr float kDefaultGgAmb = 1.6f;      // Convective loss group -> ambient (W/K)
    static constexpr float kDefaultGammaG = 0.18f;     // Water thermal equilibration fraction in group passageway
    static constexpr float kCpWater = 4184.0f;        // Specific heat capacity of water (J/(kg*K))
    static constexpr float kDefaultPmax = 1490.0f;    // Heater power at 240V UK mains (W) [1370W @ 230V]
    static constexpr float kDefaultEta = 0.95f;       // Heater coupling efficiency
    static constexpr float kDefaultDeadTime = 3.0f;   // Sensor thermal transport dead-time tau_d (s)
    static constexpr float kMaxWarmupBias = 4.5f;     // Maximum allowed boiler overheat bias during cold group soak (deg C)
    static constexpr float kOverheatCutoff = 155.0f;   // Hardware safety shutoff temperature (deg C)
    static constexpr size_t kMaxDelaySteps = 250;     // Max circular buffer length (5.0s @ 50Hz)

    ThermalObserver(float dt = 0.02f, float Pmax = kDefaultPmax, float deadTime = kDefaultDeadTime)
        : dt_(dt), Pmax_(Pmax), deadTime_(deadTime) {
        setDeadTime(deadTime);
        reset(20.0f, 20.0f);
    }

    void reset(float initialBoilerTemp = 20.0f, float initialGroupTemp = 20.0f) {
        Tb_est_ = initialBoilerTemp;
        Tg_est_ = initialGroupTemp;
        integrator_ = 0.0f;
        prevError_ = 0.0f;
        emergencyTripped_ = false;

        delayBuffer_.fill(initialBoilerTemp);
        delayIndex_ = 0;
    }

    void setDeadTime(float deadTimeSec) {
        deadTime_ = deadTimeSec;
        delaySteps_ = static_cast<size_t>(std::round(deadTime_ / dt_));
        if (delaySteps_ < 1) delaySteps_ = 1;
        if (delaySteps_ >= kMaxDelaySteps) delaySteps_ = kMaxDelaySteps - 1;
    }

    void setGains(float Kp, float Ti, float Td = 0.0f) {
        Kp_ = Kp;
        Ti_ = Ti;
        Td_ = Td;
    }

    void setInletTemp(float Tin) { Tin_ = Tin; }
    void setAmbientTemp(float Tamb) { Tamb_ = Tamb; }
    void setEfficiency(float eta) { eta_ = eta; }

    /**
     * @brief Dynamic group warmup compensation: computes the expected water temperature
     *        entering the coffee puck based on current boiler and group temperatures.
     */
    [[nodiscard]] float estimatePuckTemperature() const noexcept {
        return Tb_est_ - kDefaultGammaG * (Tb_est_ - Tg_est_);
    }

    /**
     * @brief Computes dynamic setpoint bias needed to deliver target temp to puck
     *        even when group head is cold during early warmup.
     */
    [[nodiscard]] float computeEffectiveBoilerSetpoint(float recipeTargetTemp) const noexcept {
        // T_puck = Tb - gamma_g * (Tb - Tg) = (1 - gamma_g)*Tb + gamma_g*Tg
        // Solve for Tb: Tb = (T_puck - gamma_g * Tg) / (1 - gamma_g)
        float desiredTb = (recipeTargetTemp - kDefaultGammaG * Tg_est_) / (1.0f - kDefaultGammaG);
        float bias = desiredTb - recipeTargetTemp;
        bias = std::clamp(bias, 0.0f, kMaxWarmupBias);
        return recipeTargetTemp + bias;
    }

    /**
     * @brief Computes zero-lag feed-forward electrical power (Watts) and SSR duty
     *        needed to heat incoming water at flow rate Q_raw (ml/s).
     */
    [[nodiscard]] float computeFeedForwardDuty(float flowRateMlS, float targetTemp) const noexcept {
        if (flowRateMlS <= 0.0f || Pmax_ <= 0.0f) {
            return 0.0f;
        }
        // P_ff = rho_w * c_p,w * Q * (T_set - T_in) = 4.184 * Q(ml/s) * deltaT (Watts)
        const float deltaT = std::max(0.0f, targetTemp - Tin_);
        const float P_ff = (kCpWater * 1e-3f) * flowRateMlS * deltaT;
        return std::clamp(P_ff / (Pmax_ * eta_), 0.0f, 1.0f);
    }

    /**
     * @brief Executes one 50Hz control cycle:
     *        1. Checks sensor safety invariants.
     *        2. Updates 2-Node State Observer energy balances.
     *        3. Computes Smith-predictor undelayed temperature estimate.
     *        4. Evaluates PI+D feedback + zero-lag feed-forward duty.
     *
     * @param T_sens Measured temperature from exterior boiler shell sensor (deg C).
     * @param targetTemp Desired recipe water temperature (deg C).
     * @param flowRateMlS Estimated or measured water flow through boiler (ml/s).
     * @param isBrewing Whether brew valve is open and extraction is active.
     * @return Commanded SSR duty [0.0, 1.0].
     */
    float update(float T_sens, float targetTemp, float flowRateMlS, bool isBrewing = false) {
        // --- 1. HARDWARE SAFETY INVARIANTS ---
        if (std::isnan(T_sens) || std::isinf(T_sens) || T_sens > kOverheatCutoff) {
            emergencyTripped_ = true;
            lastDuty_ = 0.0f;
            return 0.0f;
        }

        // --- 2. DYNAMIC WARMUP & INTRA-SHOT SETPOINT SHAPING ---
        const float effectiveTarget = computeEffectiveBoilerSetpoint(targetTemp);

        // --- 3. 2-NODE STATE OBSERVER ENERGY BALANCES ---
        // Mass flow rate in kg/s: Q_raw (ml/s) * 1e-3 kg/ml
        const float m_dot = std::max(0.0f, flowRateMlS) * 1e-3f;
        const float waterHeatLoss = m_dot * kCpWater * (Tb_est_ - Tin_);
        const float Q_bg = kDefaultGbg * (Tb_est_ - Tg_est_);
        const float Q_b_amb = kDefaultGbAmb * (Tb_est_ - Tamb_);
        const float Q_g_amb = kDefaultGgAmb * (Tg_est_ - Tamb_);
        const float Q_water_group = m_dot * kCpWater * kDefaultGammaG * (Tb_est_ - Tg_est_);

        // Applied heater power in previous cycle (accounting for efficiency)
        const float P_heater = eta_ * lastDuty_ * Pmax_;

        // Continuous derivatives
        const float dTb_dt = (P_heater - waterHeatLoss - Q_bg - Q_b_amb) / kDefaultCb;
        const float dTg_dt = (Q_bg + Q_water_group - Q_g_amb) / kDefaultCg;

        // Discrete Euler integration
        Tb_est_ += dTb_dt * dt_;
        Tg_est_ += dTg_dt * dt_;

        // Observer correction step: gently correct Tb_est towards T_sens to prevent long-term drift
        // (with low gain so sensor transport lag doesn't pull down state during dynamic transients)
        const float observerGain = isBrewing ? 0.005f : 0.04f;
        Tb_est_ += observerGain * (T_sens - Tb_est_);

        // --- 4. SMITH PREDICTOR DEAD-TIME COMPENSATION ---
        // Delayed model history ring buffer
        delayBuffer_[delayIndex_] = Tb_est_;
        const size_t delayedIndex = (delayIndex_ + kMaxDelaySteps - delaySteps_) % kMaxDelaySteps;
        const float Tb_delayed_model = delayBuffer_[delayedIndex];
        delayIndex_ = (delayIndex_ + 1) % kMaxDelaySteps;

        // Predicted undelayed temperature: T_sens + (Tb_model(t) - Tb_model(t - tau_d))
        const float Tb_smith = T_sens + (Tb_est_ - Tb_delayed_model);

        // --- 5. ZERO-LAG FLOW FEED-FORWARD ---
        const float u_ff = computeFeedForwardDuty(flowRateMlS, effectiveTarget);

        // --- 6. PI+D CONTROLLER WITH ANTI-WINDUP ---
        const float error = effectiveTarget - Tb_smith;

        // Proportional term
        const float p_term = Kp_ * error;

        // Derivative term on error
        const float d_term = (Td_ > 0.0f) ? (Kp_ * Td_ * (error - prevError_) / dt_) : 0.0f;
        prevError_ = error;

        // Integral accumulation with anti-windup clamping
        const float tentativeIntegral = integrator_ + (Kp_ / Ti_) * error * dt_;
        const float tentativeDuty = p_term + tentativeIntegral + d_term + u_ff;

        // Only integrate if not saturating or if error opposes saturation direction
        if ((tentativeDuty >= 0.0f && tentativeDuty <= 1.0f) ||
            (tentativeDuty > 1.0f && error < 0.0f) ||
            (tentativeDuty < 0.0f && error > 0.0f)) {
            integrator_ = tentativeIntegral;
        }

        // Clamp total command
        lastDuty_ = std::clamp(p_term + integrator_ + d_term + u_ff, 0.0f, 1.0f);
        return lastDuty_;
    }

    // Accessors
    [[nodiscard]] float getBoilerTempEstimate() const noexcept { return Tb_est_; }
    [[nodiscard]] float getGroupTempEstimate() const noexcept { return Tg_est_; }
    [[nodiscard]] float getLastDuty() const noexcept { return lastDuty_; }
    [[nodiscard]] bool isEmergencyTripped() const noexcept { return emergencyTripped_; }

  private:
    float dt_;
    float Pmax_;
    float deadTime_;
    size_t delaySteps_ = 150;

    // Default SIMC tuned gains for Smith Predictor on RI9480/14
    float Kp_ = 0.045f;
    float Ti_ = 35.0f;
    float Td_ = 0.5f;

    float Tin_ = 18.0f;   // UK mains inlet cold water temp (deg C)
    float Tamb_ = 20.0f;  // Ambient room/chassis temp (deg C)
    float eta_ = kDefaultEta;

    float Tb_est_ = 20.0f;
    float Tg_est_ = 20.0f;
    float integrator_ = 0.0f;
    float prevError_ = 0.0f;
    float lastDuty_ = 0.0f;
    bool emergencyTripped_ = false;

    std::array<float, kMaxDelaySteps> delayBuffer_{};
    size_t delayIndex_ = 0;
};

} // namespace control_physics
