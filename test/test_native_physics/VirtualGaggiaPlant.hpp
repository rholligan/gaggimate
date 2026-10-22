#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace test_sim {

/**
 * @brief High-Fidelity Software-in-the-Loop (SIL) ODE Plant Simulator for Gaggia Classic Pro (RI9480/14).
 *
 * Solves coupled non-linear differential equations representing the physical machine:
 *   - Aluminum boiler core & shell thermodynamics with heater transport delay (tau_d = 3.0s).
 *   - Conductive heat transfer to the heavy 1.15kg chromed brass group head.
 *   - Cold reservoir water influx (Tin = 18 deg C) at mass flow rate m_dot.
 *   - Reciprocating Ulka vibration pump fluid dynamics and deadhead backpressure curve.
 *   - Two-regime hydraulic compliance (air compression + elastic hose expansion).
 *   - Dynamic coffee puck resistance and channeling blowout fault injection.
 */
class VirtualGaggiaPlant {
  public:
    static constexpr float kPmax = 1490.0f;       // 240V UK mains heater power (W)
    static constexpr float kCb = 795.1f;          // Boiler thermal capacitance (J/K)
    static constexpr float kCg = 437.0f;          // Group head thermal capacitance (J/K)
    static constexpr float kGbg = 2.8f;           // Flange conduction boiler -> group (W/K)
    static constexpr float kGbAmb = 0.9f;         // Convective loss boiler -> ambient (W/K)
    static constexpr float kGgAmb = 1.6f;         // Convective loss group -> ambient (W/K)
    static constexpr float kCpWater = 4184.0f;    // Specific heat capacity of water J/(kg*K)
    static constexpr float kGammaG = 0.18f;        // Transit water thermal equilibrium fraction
    static constexpr size_t kDelaySteps = 150;    // 3.0s @ 50Hz (dt = 0.02s)

    VirtualGaggiaPlant(float dt = 0.02f) : dt_(dt) {
        reset(20.0f, 20.0f);
    }

    void reset(float initialBoilerTemp = 20.0f, float initialGroupTemp = 20.0f) {
        Tb_ = initialBoilerTemp;
        Tg_ = initialGroupTemp;
        pressure_ = 0.0f;
        Rpuck_ = 4.0f; // Default healthy puck resistance
        channelingFault_ = false;

        heaterDelayBuffer_.fill(0.0f);
        heaterDelayIndex_ = 0;

        sensorDelayBuffer_.fill(initialBoilerTemp);
        sensorDelayIndex_ = 0;
    }

    void setPuckResistance(float Rpuck) noexcept { Rpuck_ = std::max(0.05f, Rpuck); }
    void injectChannelingFault() noexcept { channelingFault_ = true; }

    /**
     * @brief Steps the physical plant simulation forward by dt (0.02s).
     *
     * @param ssrDuty Commanded heater duty [0.0, 1.0].
     * @param pumpDuty Commanded pump dimmer duty [0.0, 1.0].
     * @param valveOpen Whether 3-way solenoid valve is open (delivering to puck).
     */
    void step(float ssrDuty, float pumpDuty, bool valveOpen = true) {
        const float D_pump = std::clamp(pumpDuty, 0.0f, 1.0f);
        const float D_ssr = std::clamp(ssrDuty, 0.0f, 1.0f);

        // --- 1. HYDRAULIC & FLUID DYNAMICS ODE ---
        // Ulka pump open-flow and stall curve
        const float qMax = 12.0f * D_pump; // ml/s
        const float pStall = 15.5f * D_pump; // bar
        float Q_pump = 0.0f;
        if (pStall > 0.1f && pressure_ < pStall) {
            Q_pump = qMax * (1.0f - std::pow(pressure_ / pStall, 1.4f));
        }

        // Channeling blowout dynamics
        if (channelingFault_) {
            Rpuck_ = std::max(0.35f, Rpuck_ - 1.2f * dt_); // Rapid breakdown of puck resistance
        }

        // Puck flow based on Darcy equation: Q_puck = sqrt(P / Rpuck)
        float Q_puck = 0.0f;
        if (valveOpen && pressure_ > 0.01f) {
            Q_puck = std::sqrt(pressure_ / Rpuck_);
        }

        // Two-regime compliance: C_sys(P) = 6.0 / (P+1)^2 + 0.15 [ml/bar]
        const float C_sys = (6.0f / ((pressure_ + 1.0f) * (pressure_ + 1.0f))) + 0.15f;
        // Hydraulic mass balance: dP/dt = (Q_pump - Q_puck) / C_sys
        const float dP_dt = (Q_pump - Q_puck) / C_sys;
        pressure_ = std::clamp(pressure_ + dP_dt * dt_, 0.0f, 16.0f);

        lastPumpFlow_ = Q_pump;
        lastPuckFlow_ = Q_puck;

        // --- 2. THERMODYNAMICS ODE ---
        // Direct heater power coupling to boiler aluminum casting & water core
        const float heaterPower = D_ssr * kPmax;

        // Cold water influx cooling
        const float m_dot = (valveOpen ? Q_puck : 0.0f) * 1e-3f; // kg/s
        const float Q_water_inlet = m_dot * kCpWater * (Tb_ - Tin_);

        // Conduction to group & ambient losses
        const float Q_bg = kGbg * (Tb_ - Tg_);
        const float Q_b_amb = kGbAmb * (Tb_ - Tamb_);
        const float Q_g_amb = kGgAmb * (Tg_ - Tamb_);
        const float Q_water_group = m_dot * kCpWater * kGammaG * (Tb_ - Tg_);

        // Boiler and group head differential equations
        const float dTb_dt = (heaterPower - Q_water_inlet - Q_bg - Q_b_amb) / kCb;
        const float dTg_dt = (Q_bg + Q_water_group - Q_g_amb) / kCg;

        Tb_ += dTb_dt * dt_;
        Tg_ += dTg_dt * dt_;

        // Exterior shell sensor transport delay (sensor lagged from boiler core by tau_d)
        sensorDelayBuffer_[sensorDelayIndex_] = Tb_;
        sensorDelayIndex_ = (sensorDelayIndex_ + 1) % kDelaySteps;
        T_sens_ = sensorDelayBuffer_[sensorDelayIndex_];
    }

    [[nodiscard]] float getBoilerTemp() const noexcept { return Tb_; }
    [[nodiscard]] float getGroupTemp() const noexcept { return Tg_; }
    [[nodiscard]] float getSensorTemp() const noexcept { return T_sens_; }
    [[nodiscard]] float getPressure() const noexcept { return pressure_; }
    [[nodiscard]] float getPumpFlow() const noexcept { return lastPumpFlow_; }
    [[nodiscard]] float getPuckFlow() const noexcept { return lastPuckFlow_; }
    [[nodiscard]] float getPuckResistance() const noexcept { return Rpuck_; }

  private:
    float dt_;
    float Tb_{20.0f};
    float Tg_{20.0f};
    float T_sens_{20.0f};
    float pressure_{0.0f};
    float Rpuck_{4.0f};
    bool channelingFault_{false};

    float lastPumpFlow_{0.0f};
    float lastPuckFlow_{0.0f};

    float Tin_{18.0f};
    float Tamb_{20.0f};

    std::array<float, kDelaySteps> heaterDelayBuffer_{};
    size_t heaterDelayIndex_{0};

    std::array<float, kDelaySteps> sensorDelayBuffer_{};
    size_t sensorDelayIndex_{0};
};

} // namespace test_sim
