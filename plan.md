# GaggiMate Flow Profiling & Closed-Loop Control — Master Plan (`plan.md`)

> **Architecture:** Requirements-Driven Development (RDD)  
> **Source Base:** Fork of `jniebuhr/gaggimate` (`https://github.com/rholligan/gaggimate`)  
> **Initial Specification:** `docs/espresso_controller_comparison_gaggimate_spec.md`  
> **Target Hardware:** ESP32, 0–5V pressure transducer, AC phase-angle dimmer, Ulka vibratory pump (50Hz mains), 3-way solenoid valve, BLE scales (Bookoo, Decent, Acaia, WeighMyBru) or HX711 wired load cells.

---

## 1. Requirements Backlog

### R1. Automated Pump Characteristic Calibration Wizard
* **R1.1 Open Flow Sweep ($Q_{\text{max}} = f(\text{Duty})$):** Automated wizard routine stepping the pump through duty cycles (20%, 40%, 60%, 80%, 100%) for 5s intervals into a catch cup on a paired BLE scale to establish open-circuit flow rate $dm/dt$.
* **R1.2 Blind Basket Stall Sweep ($P_{\text{stall}} = f(\text{Duty})$):** Automated blind basket sweep recording stabilized pressure on the 0–5V transducer for each duty level ($Q = 0$).
* **R1.3 Surface Fit & Persistence:** Fit linear/polynomial pump displacement model $Q(P, \text{Duty}) = Q_{\text{max}}(\text{Duty}) \cdot \left(1 - \frac{P}{P_{\text{stall}}(\text{Duty})}\right)$ and persist coefficients / 2D LUT into ESP32 NVS, exposed via Web UI.

### R2. Real-Time Flow Estimation & Closed-Loop Profiling
* **R2.1 Real-Time Puck Flow Estimator:** Evaluate instantaneous pump output $Q_{\text{pump}}(P, \text{Duty})$ and subtract hydraulic compliance volume rate ($C \cdot \frac{dP}{dt}$) inside the controller loop to estimate water through the puck ($Q_{\text{puck}}$).
* **R2.2 Closed-Loop Flow Controller (PID / Feedback):** Replace/augment current open-loop feedforward duty calculation with an active feedback loop (PI/PID) targeting setpoint flow rate (ml/s), dynamic flow tapering, and pressure ceilings.
* **R2.3 Volumetric Integration:** Continuous accumulation of $\int Q_{\text{puck}} \, dt$ to provide virtual scale yield tracking when external hardware scales are disconnected.

### R3. Event-Driven Conditional Recipe State Machine
* **R3.1 Multi-Variable Transition Logic:** Enable compound exit conditions (`AND` / `OR`) across pressure, estimated/measured flow rate, gravimetric mass, volumetric yield, and timeout duration.
* **R3.2 Predictive Gravimetric Auto-Stop:** Model scale mass momentum $\frac{dm}{dt}$ and valve latency to cut pump power and 3-way solenoid ahead of time, hitting beverage weight targets within $\pm 0.3\text{g}$.
* **R3.3 Advanced Phase Targets:** Support linear/exponential transitions between target pressure and target flow regimes (e.g., constant 2 ml/s pre-infusion $\to$ 9 bar peak $\to$ declining 1.5 ml/s flow finish).

### R4. Headless Operation & Smart Home Telemetry
* **R4.1 Headless Core:** First-class headless ESP32 binary operation without display hardware dependency.
* **R4.2 Home Assistant / MQTT Discovery:** Auto-discovery for boiler warm-up scheduling, live extraction metrics (pressure, flow rate, puck resistance, scale weight, water volume), and recipe selection.
* **R4.3 Web UI Configuration:** Responsive local web interface for calibration wizards, recipe authoring, and real-time shot charting.

### R5. Physical & Mechanical Calibration Guards
* **R5.1 Phantom Flow & OPV Protection:** Safe pressure bounding in software to prevent reaching mechanical OPV bypass crack pressure (e.g. OPV set to 11–12 bar, software capping at 9 bar), preventing phantom flow divergence.
* **R5.2 Solenoid Coil Thermal Fade Compensation:** Thermal model tracking cumulative pump power dissipation with exponential idle cooling, applying real-time compensation scalars to $Q_{\text{max}}$ and $P_{\text{stall}}$.

---

## 2. Codebase Baseline & Gap Analysis

| Component | Upstream State (`jniebuhr/gaggimate`) | Gap / Planned Work |
| :--- | :--- | :--- |
| **`PressureController`** | Contains basic affine model ($Q_{\text{geo}} - \text{slip}$) and puck conductance state machine. | `getPumpDutyCycleForFlowRate()` is open-loop feedforward; `setFlowLimit()` / `setPressureLimit()` are no-op stubs; lacks dynamic feedback PID. |
| **Pump Calibration** | Static hard-coded coefficients in code/settings. | Needs automated BLE-assisted 2-step calibration wizard (Open Flow + Blind Stall) and NVS surface storage. |
| **Thermal Fade** | None. Coil heating causes 10–15% flow/pressure drop during sustained use. | Implement thermal energy accumulator with exponential decay constant. |
| **Recipe Engine** | Basic sequential phases with single target checks. | Refactor into compound boolean state machine (`TargetOperator::AND`, `TargetOperator::OR`). |
| **Predictive Stop** | `VolumetricRateCalculator` exists in `predictive.h` with linear regression slope. | Connect directly to BLE scale $dm/dt$ and solenoid cut-off timing. |
| **Headless Build** | Headless configs exist in `platformio.ini`. | Validate headless build parity with all new flow profiling logic. |

---

## 3. Verification & Testing Strategy

1. **Host-Side Unit Tests (`pio test -e native`):**
   - `test_pump_model`: Verify LUT interpolation, surface fit equation, and edge-case clamping.
   - `test_flow_pid`: Simulate closed-loop flow control against simulated hydraulic resistance.
   - `test_thermal_fade`: Verify thermal accumulation and decay equations over shot bursts.
   - `test_recipe_state_machine`: Unit test compound exit conditions (AND/OR, timeouts, predictive stop).
2. **End-to-End Simulation (`sim/`):**
   - Run software-in-the-loop brew simulation simulating puck saturation, channeling resistance breakdown, and scale latency.
3. **Hardware In-The-Loop Validation (HIL):**
   - Zero-backpressure flow sweep verification with physical scale.
   - Blind basket stall pressure verification with 0–5V transducer.

---

## 4. Issues, Features & Revision Log

* **[2026-10-21] Initial Setup:** Forked `jniebuhr/gaggimate` to `rholligan/gaggimate` at `C:\dev\gaggimate`. Analyzed existing `PressureController`, `predictive.h`, and `BrewProcess.h`. Master plan initiated.
* **[Pending User Spec Update]:** Awaiting anticipated revisions to `espresso_controller_comparison_gaggimate_spec.md` later this evening.
