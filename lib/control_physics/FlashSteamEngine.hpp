#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace control_physics {

/**
 * @brief Closed-Loop Pressure-Targeted Flash-Boiler Steam Controller for Gaggia Classic Pro.
 */
class FlashSteamEngine {
  public:
    enum class State : uint8_t {
        IDLE = 0,
        SUPERHEAT = 1,
        READY = 2,
        STEAMING_INJECTION = 3
    };

    static constexpr float kDefaultPmax = 1370.0f;               // Heater power (W)
    static constexpr float kDeltaHSteam = 2646.0f;               // Total enthalpy of vaporization J/g
    static constexpr float kMaxInjectionRate = 0.518f;            // Maximum sustainable vaporization rate (ml/s)
    static constexpr float kSuperheatTemp = 148.0f;              // Target superheat boiler temp (deg C)
    static constexpr float kTargetSteamPressure = 2.2f;          // Target operating steam pressure (bar gauge)
    static constexpr float kValveOpenThreshold = -0.15f;         // dP/dt threshold indicating steam wand opened (bar/s)
    static constexpr float kValveClosedPressure = 2.7f;          // Pressure recovery indicating steam wand closed (bar gauge)

    FlashSteamEngine(float dt = 0.02f) : dt_(dt) {
        reset();
    }

    void reset() noexcept {
        state_ = State::IDLE;
        integrator_ = 0.0f;
        targetPressure_ = kTargetSteamPressure;
        lastPumpDuty_ = 0.0f;
        lastHeaterDuty_ = 0.0f;
        burstTimer_ = 0.0f;
    }

    void startSteamMode() noexcept {
        state_ = State::SUPERHEAT;
        integrator_ = 0.0f;
    }

    void stopSteamMode() noexcept {
        reset();
    }

    void update(float boilerTemp, float pressureBar, float dP_dt, float &heaterDuty, float &pumpTargetFlow) noexcept {
        heaterDuty = 0.0f;
        pumpTargetFlow = 0.0f;

        switch (state_) {
        case State::IDLE:
            break;

        case State::SUPERHEAT: {
            heaterDuty = (boilerTemp < kSuperheatTemp) ? 1.0f : 0.0f;
            if (boilerTemp >= kSuperheatTemp - 0.5f) {
                state_ = State::READY;
            }
            break;
        }

        case State::READY: {
            heaterDuty = (boilerTemp < kSuperheatTemp) ? 0.75f : 0.0f;
            if (dP_dt < kValveOpenThreshold && pressureBar > 1.2f) {
                state_ = State::STEAMING_INJECTION;
                integrator_ = 0.0f;
            }
            break;
        }

        case State::STEAMING_INJECTION: {
            heaterDuty = 1.0f;

            const float error = targetPressure_ - pressureBar;
            constexpr float Kp = 0.18f;
            constexpr float Ki = 0.08f;

            integrator_ += Ki * error * dt_;
            integrator_ = std::clamp(integrator_, 0.0f, kMaxInjectionRate);

            const float rawFlowCmd = Kp * error + integrator_;
            pumpTargetFlow = std::clamp(rawFlowCmd, 0.0f, kMaxInjectionRate);

            if (pressureBar > kValveClosedPressure && dP_dt > 0.05f) {
                state_ = State::READY;
                pumpTargetFlow = 0.0f;
            }
            break;
        }
        }

        lastHeaterDuty_ = heaterDuty;
    }

    [[nodiscard]] State getState() const noexcept { return state_; }
    [[nodiscard]] float getTargetPressure() const noexcept { return targetPressure_; }

  private:
    float dt_;
    State state_{State::IDLE};
    float targetPressure_{kTargetSteamPressure};
    float integrator_{0.0f};
    float lastPumpDuty_{0.0f};
    float lastHeaterDuty_{0.0f};
    float burstTimer_{0.0f};
};

} // namespace control_physics
