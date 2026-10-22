# GaggiMate Pro Custom Firmware Master Architecture & Implementation Plan (`plan.md`)

> **Architecture:** Requirements-Driven Development (RDD) with Pure C++20 Decoupled Domain Models  
> **Source Base:** Fork of `jniebuhr/gaggimate` (`https://github.com/rholligan/gaggimate`)  
> **Specifications:**  
> 1. `docs/espresso_controller_comparison_gaggimate_spec.md` (Initial baseline)  
> 2. `docs/revised_gaggimate_pro_engineering_firmware_specification.md` (Revised engineering & physics specification)  
> **Target Hardware:** Gaggia Classic Pro (`RI9480/14`), GaggiMate Pro Kit (ESP32-S3 internal controller PCB + LilyGo T-RGB 2.1" 480×480 magnetic remote display), 0–5V pressure transducer, AC phase-angle dimmer, Ulka vibratory pump (50Hz mains), 3-way solenoid valve, BLE scales / HX711 load cells.

---

## 1. Architectural Strategy: Isolated Pure C++20 Core

To guarantee safety, eliminate ESP-IDF / Arduino RTOS coupling, and enable 100% desktop automated verification (`pio test -e native`):
```
gaggimate/
├── lib/
│   └── control_physics/          # Pure C++20 header modules (zero external framework deps)
│       ├── ThermalObserver.hpp   # 2-Node state observer, Smith predictor & flow feed-forward
│       ├── PumpSurface.hpp       # Convex P-Q-Duty surface, coil thermal fade & RLS scale adapter
│       ├── ComplianceModel.hpp   # Dynamic dP/dt volume compensation & Darcy Rpuck channeling guard
│       ├── FlashSteamEngine.hpp  # Closed-loop pressure-targeted steam injection controller
│       └── AutoCalibrator.hpp    # State machines for Hydraulic & Relay Thermal autotune wizards
├── test/
│   └── test_native_physics/      # Desktop Software-in-the-Loop (SIL) Plant Simulator
│       ├── VirtualGaggiaPlant.hpp# ODE solver (boiler, group, Ulka pump, air compliance & puck)
│       └── test_controllers.cpp  # Automated unit & E2E assertions (`pio test -e native`)
└── src/
    ├── controller/               # Thin adapter binding control_physics into 50Hz/100Hz ISR
    └── display/                  # Wireless magnetic remote power manager & Grind Advisor UI
```

---

## 2. Master Requirements Backlog

### R0. Upstream Repository Standards & Maintainer Invariants (`CONTRIBUTING.md` Compliance)
* **R0.1 Formatting & Style Enforcement:**
  - Mandatory conformance to repository [.clang-format](file:///C:/dev/gaggimate/.clang-format) (`BasedOnStyle: LLVM`, `IndentWidth: 4`, `ColumnLimit: 130`) across all C/C++ domain models, drivers, and plugins.
  - Web UI code must adhere to Prettier standards (`web/package.json`: `npm run format`).
  - Markdown documentation formatted cleanly without line noise.
* **R0.2 Static Analysis Quality Gate:**
  - Zero defects under PlatformIO Cppcheck on all target firmware environments:
    - Controller: `platformio check -e controller --fail-on-defect=medium -f "-<*>" -f "+<src/controller/>"`
    - Display: `platformio check -e display --fail-on-defect=medium -f "-<*>" -f "+<src/display/>" -f "-<src/display/ui>"`
* **R0.3 Architectural Decoupling & Dual-Node Boundaries:**
  - Controller firmware (`lib/GaggiMateController/`, `src/controller/`): Dedicated to real-time hardware control, triac zero-crossing dimmer timing, 500 SPS transducer polling, and latched safety invariants ($u_{\text{SSR}} \equiv 0$ on thermal runaway or sensor fault).
  - Display remote (`src/display/`): Coordinates UI, state machines, Wi-Fi, MQTT, and Home Assistant discovery strictly through the event-driven `PluginManager` architecture.
  - BLE Communication: Adheres strictly to Protocol Buffers schema defined in `lib/NanoPbComm/proto/gaggimate.proto`.
  - Pure Physics Library (`lib/control_physics/`): Isolated C++20 domain headers completely decoupled from Arduino/ESP-IDF RTOS for 100% host desktop testing.
* **R0.4 Target Build Integrity:**
  - Every commit must produce clean, non-regressing compilation for all PlatformIO targets: `controller`, `display-headless`, and `display`.
* **R0.5 Contributor License Agreement (CLA) Tracking:**
  - Formal tracking of CLA compliance requiring contact with `@jniebuhr` (`mdwasp` on Discord) prior to submitting upstream Pull Requests from `rholligan/gaggimate` to `jniebuhr/gaggimate`.

### R1. Front-Loaded Thermodynamic Model & Thermal Control (`ThermalObserver.hpp`)
* **R1.1 Two-Node State-Space Observer:**
  * Modeled with boiler node ($C_b \approx 795.1\text{ J/K}$) and brass group head node ($C_g \approx 437.0\text{ J/K}$).
  * Estimates true water temperature $T_b(t)$ and unmeasured group temperature $T_g(t)$ at $50\text{ Hz}$.
  * Conductive flange conductance $G_{b \to g} \approx 2.8\text{ W/K}$; group transit fraction $\gamma_g \approx 0.18$.
* **R1.2 Dynamic Warmup Offset Compensation:**
  * Computes puck water entry temperature: $\hat{T}_{\text{puck}}(t) = \hat{T}_b(t) - \gamma_g (\hat{T}_b(t) - \hat{T}_g(t))$.
  * Automatically injects up to $+4.5^\circ\text{C}$ overheat bias on cold boot, decaying to nominal as $\hat{T}_g$ soaks.
* **R1.3 Zero-Lag Flow Feed-Forward ($u_{\text{ff}}$):**
  * Instantaneous power injection: $P_{\text{ff}}(t) = 4.184 \cdot Q_{\text{raw}}(t) \cdot (T_{\text{set}}(t) - T_{\text{in}})$.
  * Injects corresponding SSR duty $u_{\text{ff}} = \text{clamp}(P_{\text{ff}} / P_{\text{max}}, 0, 1)$ without sensor transport delay.
* **R1.4 Smith Predictor Dead-Time Compensation:**
  * Compensates for external shell sensor dead time ($\tau_d \approx 2.5\text{--}4.0\text{ s}$) using an internal delay ring buffer:
    $\tilde{T}_b(t) = T_{\text{sens}}(t) + (\hat{T}_{b,\text{model}}(t) - \hat{T}_{b,\text{model}}(t - \tau_d))$.
* **R1.5 Intra-Shot Thermal Profiling:**
  * Support active intra-shot temperature curves (e.g. declining $94^\circ\text{C} \to 88^\circ\text{C}$ for bitter tannin reduction; ascending $88^\circ\text{C} \to 94^\circ\text{C}$).

### R2. Non-Linear Fluid Dynamics & Hydraulic Compliance (`PumpSurface.hpp`, `ComplianceModel.hpp`)
* **R2.1 Non-Linear Convex Pump Surface:**
  * Empirical model: $Q_{\text{raw}}(P, D, E_{\text{coil}}) = \alpha_{\text{coil}}(E_{\text{coil}}) \cdot Q_{\text{max}}(D) \cdot \left[1 - \left(\frac{\min(P, P_{\text{stall}}'(D))}{P_{\text{stall}}'(D)}\right)^{\kappa(D)}\right]$.
  * Fitted quadratics for $Q_{\text{max}}(D)$ and $P_{\text{stall}}'(D)$, with convexity exponent $\kappa(D) \in [1.25, 1.65]$.
* **R2.2 Solenoid Coil Thermal Fade Model:**
  * Dynamic energy accumulator: $\frac{dE_{\text{coil}}}{dt} = D^2 - \frac{E_{\text{coil}}}{\tau_{\text{cool}}}$ with $\tau_{\text{cool}} \approx 95\text{ s}$ and $\beta_{\text{fade}} \approx 0.0028\text{ s}^{-1}$.
  * Scales $Q_{\text{max}}$ and $P_{\text{stall}}$ by $\alpha_{\text{coil}} = 1.0 - \beta_{\text{fade}} \cdot E_{\text{coil}}$.
* **R2.3 Dynamic Hydraulic Compliance Observer:**
  * Separates true puck flow from pump volume: $Q_{\text{puck}}(t) = \text{clamp}(Q_{\text{raw}}(t) - C_{\text{sys}}(P) \frac{dP}{dt}, 0, Q_{\text{max}}(1.0))$.
  * Two-regime compliance model: $C_{\text{sys}}(P) = \frac{V_{\text{air},0}}{(P + 1.0)^2} + c_{\text{hose}}$.
* **R2.4 Real-Time Darcy Puck Resistance & Auto-Save Channeling Guard:**
  * Computes $R_{\text{puck}}(t) = \frac{P(t)}{Q_{\text{puck}}(t)^2 + \epsilon_0}$.
  * If $\frac{d\ln R_{\text{puck}}}{dt} < -0.18\text{ s}^{-1}$ during extraction (channel blowout detected), instantly triggers protective flow-capped taper ($Q_{\text{target}} = 1.8\text{ ml/s}$).
* **R2.5 Closed-Loop RLS Scale Calibration:**
  * Recursive Least Squares parameter update ($\lambda = 0.05$) during steady pour against external BLE scale $\frac{dm}{dt}$.
* **R2.6 Acoustic Half-Wave Skipping:**
  * Bresenham integral half-wave skipping during low-pressure pre-infusion ($P < 2.5\text{ bar}$) for silent operation.

### R3. Automated Zero-Expertise Calibration Wizards (`AutoCalibrator.hpp`)
* **R3.1 Wizard 1: Hydraulic & Compliance Calibrator (3 Minutes):**
  * Stage A (Open Flow): Steps $D \in [0.25, 0.40, 0.55, 0.70, 0.85, 1.00]$ for 4s into BLE scale; fits $Q_{\text{max}}(D)$.
  * Stage B (Blind Basket): Runs $D=0.60$ to solve for $V_{\text{air},0}$ and $c_{\text{hose}}$ from $\frac{Q_{\text{raw}}}{dP/dt}$; checks OPV crack threshold; steps $D$ to record $P_{\text{stall}}(D)$.
* **R3.2 Wizard 2: Relay-Feedback Thermal Autotuner (8 Minutes):**
  * Stage A (Åström-Hägglund Limit-Cycle): Toggles SSR ($d=0.50$) around $88^\circ\text{C}$ with $\epsilon = 0.3^\circ\text{C}$ for 3 cycles. Computes $K_u$, $T_u$, $\tau_d$, and non-overshooting SIMC PI gains for Smith Predictor.
  * Stage B (Cold-Water Disturbance): 100% open pump flow for 15s to calibrate heater efficiency $\eta$ and inlet $T_{\text{in}}$.

### R4. Flash-Boiler Steam Engine (`FlashSteamEngine.hpp`)
* **R4.1 Thermodynamic Influx Ceiling:**
  * Enforces maximum continuous vaporization limit: $\dot{m}_{\text{flash,max}} = \frac{1370\text{ W}}{2646\text{ J/g}} \approx 0.518\text{ g/s}$ ($Q_{\text{raw}} \le 0.52\text{ ml/s}$).
* **R4.2 Closed-Loop Pressure Injection:**
  * Pre-purges and superheats boiler to $148^\circ\text{C}$ ($3.5\text{ bar}$).
  * Detects knob opening via pressure drop rate ($\frac{dP}{dt} < -0.15\text{ bar/s}$).
  * PI-modulated 2Hz pump phase-angle bursts maintaining continuous $P_{\text{steam,set}} = 2.2\text{ bar}$.

### R5. Wireless Magnetic Display Remote (LilyGo T-RGB) & Grind Advisor
* **R5.1 Thermal Isolation & Magnetic Enclosure:**
  * 3D printed ASA/ABS/PC housing with 5.0mm ventilated standoff + 2.0mm silicone foam thermal barrier.
  * 1200 mAh LiPo connected via onboard 1.25mm JST battery connector.
* **R5.2 Display Power Management State Machine:**
  * Active (190mA @ 60Hz LVGL) $\to$ Dimmed Idle (45mA @ 80MHz CPU) $\to$ Deep Sleep (<1.2mA, wake-on-touch via FT3267 `TP_INT` in <350ms).
* **R5.3 Post-Shot Automated Grind Advisor:**
  * Evaluates $t_{\text{compress}}$ and mean $\bar{R}_{\text{puck}}$ to recommend grind adjustments: $\Delta_{\text{grind}} = K_{\text{grind}} \ln(\bar{R}_{\text{actual}} / \bar{R}_{\text{target}})$.

### R6. Desktop Software-in-the-Loop (SIL) Plant Simulator (`test/test_native_physics/`)
* **R6.1 `VirtualGaggiaPlant.hpp`:** ODE plant solver simulating RI9480/14 thermal, hydraulic, and puck dynamics.
* **R6.2 `test_controllers.cpp`:** Full assertion suite executed with `pio test -e native` verifying safety invariants, zero-droop thermal response, compliance isolation, and channeling auto-save.

---

## 3. Staged Rollout & Delivery Roadmap

* [x] **Milestone 0: Baseline Setup & Upstream Standards (`CONTRIBUTING.md`):**
  - Master plan, repository fork, and initial hardware audit.
  - Verification of upstream coding standards, `.clang-format` enforcement, and Cppcheck static analysis with 0 defects.
  - CLA requirements and dual-MCU architectural invariants established as non-negotiable project baselines.
* [x] **Milestone 1: Core Physics Library (`lib/control_physics/`):**
  - Implemented `ThermalObserver.hpp` (2-node lumped capacitance, dynamic warmup bias, zero-lag feed-forward, Smith predictor).
  - Implemented `PumpSurface.hpp` (Convex Ulka model, coil thermal fade accumulator, RLS scale adaptation).
  - Implemented `ComplianceModel.hpp` (Two-regime compliance observer, Darcy puck resistance, channeling Auto-Save guard).
  - Implemented `FlashSteamEngine.hpp` (Thermodynamic continuous injection ceiling $\le 0.518\text{ ml/s}$, closed-loop PI pressure maintenance).
  - Implemented `AutoCalibrator.hpp` (Hydraulic compliance wizard & Åström-Hägglund relay thermal autotuner with SIMC gains).
* [x] **Milestone 2: Desktop SIL Plant Simulator (`test/test_native_physics/`):**
  - Implemented `VirtualGaggiaPlant.hpp` (ODE plant solver simulating coupled thermodynamics, heat conduction to brass group head, cold water influx, Ulka pump curves, hydraulic compliance, and puck bed channeling fault injection).
  - Implemented `test_controllers.cpp` (16 exhaustive tests: zero-droop feedforward, Smith predictor dead-time stability, compliance isolation, channeling auto-save, SSR fail-open safety invariants).
  - 100% pass rate achieved under `pio test -e native -f test_native_physics`.
* [x] **Milestone 3: Controller Firmware Hookup (`src/controller/`):**
  - Integrated `PhysicsControlCoordinator.{h,cpp}` connecting pure C++20 domain models to 50Hz/100Hz hardware timer ISR with fail-open sensor guards and zero-lag feed-forward.
  - Implemented Bresenham sub-harmonic wave skipping in `DimmedPump.{h,cpp}` (`shapeBresenhamPreinfusion`, 12.5Hz on 50Hz mains for $P < 2.5\text{ bar}$).
  - Validated clean compilation on ESP32-S3 (`pio run -e controller`: RAM 12.1%, Flash 20.8%).
* [x] **Milestone 4: Web UI & Calibration Wizards:**
  - Integrated `CalibrationManager.{h,cpp}` orchestrating 2-step Open Flow & Blind Stall sweeps.
  - Linked `PumpSurfaceModel.{h,cpp}` for 2D LUT interpolation and bisection feedforward duty solving.
  - Added REST/WebSocket/MQTT telemetry endpoints for real-time monitoring and HA autodiscovery.
* [x] **Milestone 5: Wireless Magnetic Display Remote & Power Management:**
  - Implemented `PowerManager.{h,cpp}`: 3-state power management (Active 190mA @ 240MHz $\to$ Dimmed 45mA @ 80MHz $\to$ Deep Sleep <1.2mA with FT3267 touch wake on RTC GPIO 1 in <350ms).
  - Implemented `GrindAdvisor.h`: Post-shot Darcy puck resistance analyzer computing logarithmic grind offset advice ($\Delta_{\text{grind}} = K_{\text{grind}} \ln(\bar{R}_{\text{actual}} / \bar{R}_{\text{target}})$).
  - Validated clean compilation on LilyGo T-RGB (`pio run -e display`: RAM 29.1%, Flash 47.8%; `pio run -e display-headless`: RAM 24.7%, Flash 33.3%).

---

## 4. Verification & Testing Log
* **Upstream Standards & Static Analysis:**
  - `clang-format`: Formatted with `.clang-format` (`BasedOnStyle: LLVM`, `IndentWidth: 4`, `ColumnLimit: 130`).
  - `pio check -e controller`: **PASSED (0 defects found)**
  - `pio check -e display`: **PASSED (0 defects found)**
* **Host Tests (`pio test -e native`):**
  - `test_native_physics`: **16/16 PASSED** (Desktop SIL Plant Simulator & Pure C++20 Domain Library)
  - `test_pump_surface_model`: **16/16 PASSED** (Pump surface fit, nodal interpolation, bisection inversion)
  - `test_adversarial_m1`: **16/16 PASSED** (Extreme pressures, out-of-range duties, non-monotonic rejection, rollover)
  - `test_e2e_requirements`: **65/65 PASSED** (Tiers 1-4 full feature, boundary, combination, and application scenarios)
  - **Total: 113 / 113 Test Cases Succeeded (100% Pass Rate)**
* **Target Embedded Builds:**
  - `pio run -e controller`: **SUCCESS** (ESP32-S3, RAM: 12.1% [39.5 KB], Flash: 20.8% [696.1 KB])
  - `pio run -e display-headless`: **SUCCESS** (ESP32-S3, RAM: 24.7% [80.8 KB], Flash: 33.3% [2.18 MB])
  - `pio run -e display`: **SUCCESS** (LilyGo T-RGB, RAM: 29.1% [95.5 KB], Flash: 47.8% [3.13 MB])
