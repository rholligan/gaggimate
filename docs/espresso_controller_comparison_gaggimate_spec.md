# Espresso Controller Comparison & Custom GaggiMate Firmware Specification

## 1. Objectives & Requirements
* **Primary Goal:** Automated, repeatable straight black espresso extraction with minimal daily manual intervention once configured.
* **Form Factor:** Headless operation (clean stainless steel counter aesthetic with no front-mounted screen required for daily operation).
* **Ecosystem Integration:** Local Web UI configuration and native Home Assistant / MQTT integration for automated warm-up schedules and telemetry.
* **Control Targets:** PID boiler temperature stability, automated low-pressure pre-infusion, closed-loop pressure and flow profiling, conditional phase transitions, and gravimetric (stop-by-weight) auto-stop.

---

## 2. Controller Ecosystem Comparison

| Feature / Capability | Gaggiuino (Gen 3) | GaggiMate (Pro Kit) | CleverCoffee |
| :--- | :--- | :--- | :--- |
| **Target Hardware Base** | Gaggia Classic, Rancilio Silvia | Gaggia Classic, Rancilio Silvia | Rancilio Silvia, Gaggia Classic |
| **Microcontroller** | STM32 / Custom PCB | ESP32 | ESP32 |
| **Source Model** | Closed-source (Gen 3 hardware-locked) | Open-source (C++ / PlatformIO) | Open-source (C++ / Arduino) |
| **Headless Operation** | No (tightly coupled to touchscreen) | Native (dedicated headless firmware binary) | Yes (minimal OLED or headless web UI) |
| **Home Assistant / MQTT** | Limited / Unofficial | Native MQTT & mDNS discovery | Native MQTT & Home Assistant integration |
| **PID Temperature Control** | Yes | Yes | Yes |
| **Closed-Loop Pressure Profiling** | Yes ($0\text{--}5\text{V}$ transducer + AC dimmer) | Yes ($0\text{--}5\text{V}$ transducer + AC dimmer) | No (timed relay pulses only) |
| **Dynamic Flow Profiling** | Yes (mathematical pump model) | Not in stock firmware (software extendable) | No |
| **Gravimetric (Stop-by-Weight)** | Integrated drip-tray load cells + BLE scales | Native BLE scales (Bookoo, Decent, Acaia) | Basic BLE scale / timed stop |
| **Recipe State Machine** | Multi-variable conditional triggers | Time-based pressure ramps (stock) | Fixed timer pre-infusion and brew |

### Platform Selection Summary
* **Why Gaggiuino leads out of the box:** Four years of iterative fluid-dynamics modeling on the Ulka vibration pump, providing real-time flow estimation ($\text{ml/s}$) and complex conditional recipe steps. However, Gen 3 is closed-source, locked to proprietary display hardware, and lacks native headless or Home Assistant integration.
* **Why GaggiMate is the ideal foundation:** Built on standard ESP32 hardware with an open C++ / PlatformIO codebase (`jniebuhr/gaggimate`), native headless firmware support, WebSocket Web UI, MQTT discovery, and BLE scale integration. Because the GaggiMate Pro kit includes the exact same physical sensor and actuator suite as Gaggiuino ($0\text{--}5\text{V}$ pressure transducer and AC phase-angle dimmer), all missing Gaggiuino physics features can be implemented purely in software.

---

## 3. Hardware Architecture Audit (GaggiMate Pro)

No hardware blockers exist on the GaggiMate Pro kit for full feature parity with Gaggiuino:
* **Microcontroller:** ESP32 with built-in Wi-Fi and Bluetooth Low Energy (BLE).
* **Pressure Sensing:** $0\text{--}5\text{V}$ inline pressure transducer measuring real-time hydraulic pressure at the boiler/group.
* **Pump Actuation:** AC phase-angle dimmer module driven by an ESP32 hardware timer, enabling granular stroke power control on the Ulka solenoid pump.
* **Thermal Control:** Solid State Relay (SSR) driving the boiler heating elements via a PID loop.
* **Weight Sensing:**
  * **Option A (Zero Hardware Mod):** Native BLE connection to an external espresso scale (e.g., WeighMyBru, Bookoo Themis, Decent Scale, Acaia Lunar).
  * **Option B (Internal Load Cells):** Wired strain gauges under the drip tray routed through an HX711 amplifier to two spare GPIO pins (Clock and Data) on the ESP32 expansion pads.

---

## 4. Custom Firmware Specification (GaggiMate Fork)

### Module 1: Automated Pump Characteristic Calibration Loop
Ulka vibration pumps have wide manufacturing tolerances ($\pm 15\%$) and behave differently on $50\text{Hz}$ UK mains vs. $60\text{Hz}$ US mains. Instead of relying on static hard-coded lookup tables, an automated calibration wizard maps the specific pump's Pressure-Flow-Duty surface using a connected BLE scale as ground truth.

1. **Step 1: Open Flow Sweep (Zero Backpressure)**
   * **Setup:** Portafilter removed; empty catch cup placed on the paired BLE scale.
   * **Execution:** Firmware steps the pump dimmer through discrete phase-cut duty percentages (e.g., $20\%, 40\%, 60\%, 80\%, 100\%$) for $5\text{ seconds}$ per step.
   * **Output:** Reads scale mass rate of change ($\frac{dm}{dt}$) to establish maximum open-circuit flow as a function of dimmer duty:
     $$Q_{\text{max}} = f(\text{Duty})$$

2. **Step 2: Deadhead Stall Sweep (Zero Flow)**
   * **Setup:** Portafilter inserted with a blind backflush basket (zero water exit).
   * **Execution:** Firmware steps through the same dimmer duty percentages while polling the $0\text{--}5\text{V}$ pressure transducer until pressure stabilizes at each step.
   * **Output:** Establishes stall pressure as a function of dimmer duty where flow is zero ($Q = 0$):
     $$P_{\text{stall}} = f(\text{Duty})$$

3. **Step 3: Surface Fit & NVS Storage**
   * **Math Model:** Models the reciprocating solenoid pump as a linear displacement device throttled by backpressure:
     $$Q(P, \text{Duty}) = Q_{\text{max}}(\text{Duty}) \cdot \left(1 - \frac{P}{P_{\text{stall}}(\text{Duty})}\right)$$
   * **Persistence:** Stores the resulting 2D Look-Up Table (LUT) or polynomial coefficients in the ESP32 non-volatile storage (NVS), exposed via the Web UI for recalibration.

### Module 2: Real-Time Flow Estimator & Volumetric Integrator
* **Control Loop Execution:** Inside the $50\text{Hz}/100\text{Hz}$ interrupt service routine, poll current transducer pressure $P$ and active dimmer $\text{Duty}$, then query the calibrated LUT to compute real-time estimated puck flow $Q$ in $\text{ml/s}$.
* **Closed-Loop Flow Profiling:** Add a PID/PI controller mode that adjusts dimmer $\text{Duty}$ dynamically to maintain a target flow rate (e.g., $1.5\text{ ml/s}$ during blooming or tapering flow if puck resistance drops due to channeling).
* **Volumetric Integration:** Accumulate $\int Q \, dt$ across the shot cycle to estimate total water volume delivered and support volumetric phase triggers when a scale is not active.

### Module 3: Event-Driven Conditional Recipe Engine
Refactor the stock time-based profile execution engine into a state machine supporting multi-variable transition conditions and predictive cut-off:
* **Phase Parameters:** Target Control Mode (Pressure or Flow), Target Setpoint ($\text{bar}$ or $\text{ml/s}$), Ramp Rate (linear or exponential curve), and Maximum Phase Duration (timeout safety).
* **Compound Exit Triggers:** Allow any phase to advance to the next step based on logical `OR` / `AND` evaluations (e.g., exit pre-infusion when $\text{Pressure} \ge 3.5\text{ bar}$ `OR` $\text{Scale Weight} \ge 3.0\text{g}$ `OR` $\text{Time} \ge 10\text{s}$).
* **Predictive Gravimetric Auto-Stop:** Track scale $\frac{dm}{dt}$ momentum and cut power to the pump and 3-way solenoid valve early so in-flight liquid lands precisely on the target beverage weight (e.g., $36.0\text{g}$).

---

## 5. Physical & Mechanical Calibration Guards

To ensure the mathematical flow model matches physical reality inside the machine, two mechanical factors must be accounted for:

1. **Eliminating OPV Bypass ("Phantom Flow")**
   * **Issue:** If the mechanical Over-Pressure Valve (OPV) opens at $9\text{ bar}$, water diverts back into the tank before reaching the group head. The pressure transducer still reads $9\text{ bar}$, causing the firmware to overestimate flow through the coffee puck.
   * **Mitigation:** Set the mechanical OPV spring higher ($11\text{ to }12\text{ bar}$) so it acts purely as an emergency safety valve. Use the software pressure controller and AC dimmer to cap brewing pressure electronically at $9\text{ bar}$, ensuring $100\%$ of pumped water passes through the puck.

2. **Solenoid Coil Thermal Fade Compensation**
   * **Issue:** As the Ulka pump runs, its copper winding heats up, increasing electrical resistance and reducing magnetic plunger force by $10\%$ to $15\%$ over extended or back-to-back shots.
   * **Mitigation:** Implement a thermal accumulator variable in firmware that tracks cumulative pump duty over time with an exponential cooling decay constant when idle, applying a proportional compensation scalar to $Q_{\text{max}}(\text{Duty})$ and $P_{\text{stall}}(\text{Duty})$.