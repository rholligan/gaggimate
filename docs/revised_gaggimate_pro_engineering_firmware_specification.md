# GaggiMate Pro Custom Firmware & Hardware Specification (RI9480/14)

## 1. Executive Summary & Multi-Disciplinary Architecture Audit

### 1.1 Objectives & Operational Philosophy

* **Primary Goal:** Achieve and exceed Gaggiuino Gen 3 extraction quality and steam performance on a Gaggia Classic Pro (`RI9480/14`) while retaining a clean aesthetic, full open-source ownership, and native Home Assistant integration.

* **Staged Rollout Strategy:** Install the GaggiMate Pro hardware kit and run stock GaggiMate Pro firmware on Day 1 for immediate PID control, basic pressure/flow profiling, and BLE scale connectivity. Incrementally develop, simulate, and flash custom physics and control modules using an AI-assisted (Claude) PlatformIO workflow.

* **Form Factor:** Internal GaggiMate Pro controller PCB paired with a detachable, battery-powered wireless magnetic display remote (LilyGo T-RGB) and headless Web UI / Home Assistant automation.

### 1.2 Specialist Sub-Agent Audit Findings

To eliminate implementation bottlenecks, the system architecture has been audited across four engineering domains:

1. **Thermodynamics & Heat Transfer:** Stock single-loop PID controllers fail during extraction on the Gaggia Classic Pro because the temperature sensor is mounted on the exterior aluminum shell, creating a $2.5\text{ to }4.0\text{ s}$ thermal transport delay ($\tau_d$) before cold incoming water is detected. Furthermore, the heavy brass group head lags behind the aluminum boiler during warmup. **Solution:** Replace standard PID with a **2-Node Lumped-Capacitance Thermal State Observer** coupled with **Zero-Lag Flow Feed-Forward Control** and a **Smith Predictor**.

2. **Fluid Dynamics & Control Theory:** Stock GaggiMate Pro supports multi-phase JSON profiles with pressure and flow targets, but relies on an uncalibrated pump lookup model and ignores system volume expansion under pressure. **Solution:** Implement a **3-parameter non-linear convex Ulka pump model**, a **Dynamic Hydraulic Compliance Observer**, and **Adaptive Puck Resistance (**$R_{\text{puck}}$**) Control**.

3. **Hardware & Industrial Design:** In the GaggiMate architecture, the LilyGo T-RGB display is already an independent ESP32-S3 node that communicates with the internal controller PCB wirelessly via BLE, using a 2-wire harness strictly for $5\text{V}$ power. Converting it into a true wireless magnetic remote is straightforward, provided thermal isolation (protecting the LiPo cell from $60^\circ\text{C}$ chassis heat) and deep-sleep power management are engineered into the enclosure and display firmware.

4. **Embedded Software Engineering (AI/Claude Workflow):** Prompting an LLM to modify monolithic RTOS/hardware-coupled firmware directly risks regressions and thermal runaway bugs. **Solution:** Isolate all thermodynamic, fluid-dynamic, and auto-tuning math into pure, hardware-agnostic C++20 header modules (`lib/control_physics/`) verified against a desktop **Software-in-the-Loop (SIL) Gaggia Plant Simulator** (`pio test -e native`) before flashing the ESP32.

## 2. Ecosystem & Capability Comparison

| **Feature / Capability** | **Gaggiuino (Gen 3)** | **Stock GaggiMate Pro** | **Custom GaggiMate Pro (This Spec)** | 
| **Controller / Display MCU** | STM32 + Wired Screen | ESP32-S3 PCB + Wireless ESP32-S3 T-RGB | ESP32-S3 PCB + Battery Magnetic T-RGB | 
| **Source Code & Extensibility** | Closed-source (hardware-locked) | Open-source (C++ / PlatformIO) | Open-source + Native SIL Plant Simulator | 
| **Boiler Thermal Control** | PID + basic flow compensation | Standard single-loop PID | **2-Node State Observer + Smith Predictor + Exact Flow Feed-Forward** | 
| **Intra-Shot Temp Profiling** | Limited | Static single target per shot | **Dynamic multi-phase thermal ramps (**$94^\circ\text{C} \to 88^\circ\text{C}$**)** | 
| **Pump Flow Modeling** | Calibrated empirical polynomial | Static generic lookup table | **Auto-calibrated convex surface + coil thermal fade + RLS scale learning** | 
| **Hydraulic Compliance (**$\Delta V$**)** | Static offset | Uncompensated | **Dynamic** $\frac{dP}{dt}$ **compliance observer** | 
| **Channeling Mitigation** | Manual profile design | None | **Real-time Darcy resistance (**$R_{\text{puck}}$**) "Auto-Save" limiter** | 
| **Steam Architecture** | Open-loop pulsed pump ("Dream Steam") | Static PID thermostat | **Closed-loop pressure-targeted flash-boiler injection** | 
| **Calibration Requirement** | Fixed hardware assumptions | Manual PID tuning | **Automated 3-minute Hydraulic & Relay-Feedback Thermal Wizards** | 

## 3. Front-Loaded Thermodynamic Model & Thermal Control

### 3.1 Physical Parameters of the RI9480/14 Boiler Assembly

The Gaggia Classic Pro uses an externally heated dry-cast aluminum block mated to a chromed brass group head:

* **Water Core Volume (**$V_w$**):** $100\text{ ml}$ ($m_w \approx 0.100\text{ kg}$, specific heat $c_{p,w} = 4184\text{ J/(kg}\cdot\text{K)}$, thermal capacitance $C_w = m_w c_{p,w} \approx 418.4\text{ J/K}$).

* **Aluminum Boiler Shell (**$m_{\text{Al}}$**):** $\approx 0.42\text{ kg}$ ($c_{p,\text{Al}} = 897\text{ J/(kg}\cdot\text{K)}$, $C_{\text{Al}} \approx 376.7\text{ J/K}$).Combined effective boiler node capacitance:
  

  $$
  C_b = C_w + C_{\text{Al}} \approx 795.1\text{ J/K}
  $$

* **Heater Power (**$P_{\text{max}}$**):** Two external pressed cartridge elements rated at $2 \times 685\text{ W} = 1370\text{ W}$ at $230\text{ V}$ ($1490\text{ W}$ at $240\text{ V}$ UK mains).

* **Brass Group Head & Portafilter (**$m_g$**):** $\approx 1.15\text{ kg}$ ($c_{p,\text{brass}} = 380\text{ J/(kg}\cdot\text{K)}$, thermal capacitance $C_g \approx 437.0\text{ J/K}$).

### 3.2 Two-Node State-Space Thermal Observer

Because only the boiler shell temperature $T_{\text{sens}}(t)$ is measured directly, a discrete-time state observer running at $\Delta t = 0.02\text{ s}$ ($50\text{ Hz}$) estimates both the true internal boiler water temperature $T_b(t)$ and the unmeasured brass group head temperature $T_g(t)$:

1. **Boiler Node Energy Balance:**
   

   $$
   C_b \frac{dT_b}{dt} = \eta P_{\text{SSR}}(t - \tau_d) - \dot{m}(t) c_{p,w} \left(T_b(t) - T_{\text{in}}\right) - G_{b \to g}\left(T_b(t) - T_g(t)\right) - G_{b \to \text{amb}}\left(T_b(t) - T_{\text{amb}}\right)
   $$

2. **Group Head Node Energy Balance:**
   

   $$
   C_g \frac{dT_g}{dt} = G_{b \to g}\left(T_b(t) - T_g(t)\right) + \dot{m}(t) c_{p,w} \gamma_g \left(T_b(t) - T_g(t)\right) - G_{g \to \text{amb}}\left(T_g(t) - T_{\text{amb}}\right)
   $$

Where:

* $\dot{m}(t) = \rho_w Q_{\text{raw}}(t) \times 10^{-3}\text{ kg/s}$ is the instantaneous mass flow rate of cold reservoir water ($T_{\text{in}} \approx 18^\circ\text{C}$ in the UK) entering the boiler.

* $G_{b \to g} \approx 2.8\text{ W/K}$ is the conductive thermal conductance between the boiler flange and brass group head.

* $G_{b \to \text{amb}} \approx 0.9\text{ W/K}$ and $G_{g \to \text{amb}} \approx 1.6\text{ W/K}$ are convective heat loss coefficients to the chassis/room.

* $\gamma_g \approx 0.18$ represents the fraction of thermal equilibrium reached as brew water transits the group head passageways.

**Dynamic Warmup Offset Compensation:**
When the machine is turned on cold, $T_b$ reaches $93^\circ\text{C}$ in $\approx 3.5\text{ minutes}$, while the heavy brass group $T_g$ is still at $\approx 68^\circ\text{C}$. Pulling a shot immediately results in sour, under-extracted espresso because the cold group strips heat from the water. Using the observer's real-time estimate of $\hat{T}_g(t)$, the controller computes the true puck water entry temperature:

$$
\hat{T}_{\text{puck}}(t) = \hat{T}_b(t) - \gamma_g \left(\hat{T}_b(t) - \hat{T}_g(t)\right)
$$


During early warmup, the firmware automatically elevates the boiler setpoint $T_{b,\text{set}}$ (up to $+4.5^\circ\text{C}$ overheat bias) so that $\hat{T}_{\text{puck}}$ hits the exact recipe target even if a shot is pulled only $5\text{ minutes}$ after boot, decaying smoothly to normal offset as $\hat{T}_g$ reaches thermal soak.

### 3.3 Zero-Lag Flow-Coupled Feed-Forward + Smith Predictor

To prevent intra-shot temperature droop caused by sensor dead time ($\tau_d \approx 3.0\text{ s}$):

1. **Instantaneous Flow Feed-Forward (**$u_{\text{ff}}$**):**
   The exact thermal power required to heat incoming cold water at flow rate $Q_{\text{raw}}(t)$ ($\text{ml/s}$) from $T_{\text{in}}$ to $T_{\text{set}}(t)$ is injected directly into the SSR command with zero delay:
   

   $$
   P_{\text{ff}}(t) = \rho_w c_{p,w} Q_{\text{raw}}(t) \left(T_{\text{set}}(t) - T_{\text{in}}\right) = 4.184 \cdot Q_{\text{raw}}(t) \cdot \left(T_{\text{set}}(t) - T_{\text{in}}\right)
   $$

   $$
   u_{\text{ff}}(t) = \text{clamp}\left(\frac{P_{\text{ff}}(t)}{P_{\text{max}}},\; 0.0,\; 1.0\right)
   $$

   
   *Example:* At $Q_{\text{raw}} = 2.5\text{ ml/s}$, $T_{\text{set}} = 93^\circ\text{C}$, and $T_{\text{in}} = 18^\circ\text{C}$, $P_{\text{ff}} = 4.184 \times 2.5 \times 75 = 784.5\text{ W}$ ($u_{\text{ff}} = 57.3\%$ SSR duty applied the exact millisecond the pump engages).

2. **Smith Predictor Dead-Time Compensation:**
   A circular buffer of length $N_d = \lfloor \tau_d / \Delta t \rfloor$ stores historical heater commands. The feedback PI+D controller acts on the predicted undelayed temperature $\tilde{T}_b(t)$:
   

   $$
   \tilde{T}_b(t) = T_{\text{sens}}(t) + \left(\hat{T}_{b,\text{model}}(t) - \hat{T}_{b,\text{model}}(t - \tau_d)\right)
   $$

   
   Total SSR duty command:
   

   $$
   u_{\text{SSR}}(t) = \text{clamp}\left(u_{\text{PI}}(t) + u_{\text{ff}}(t),\; 0.0,\; 1.0\right)
   $$

### 3.4 Intra-Shot Dynamic Temperature Profiling ("Thermal Surfing by Wire")

Because the $100\text{ ml}$ boiler has a high ratio of water throughput to boiler volume ($36\text{ g}$ out + $18\text{ g}$ retained in puck $\approx 54\text{ ml}$, or $>50\%$ boiler turnover in a single shot), we can execute active intra-shot temperature curves:

* **Declining Profile (**$94^\circ\text{C} \to 88^\circ\text{C}$**):** Pre-soaks the boiler at $94^\circ\text{C}$ for high-solubility pre-infusion, then sets $u_{\text{SSR}} = 0$ during the second half of extraction, using the calibrated cold-water influx rate $Q_{\text{raw}}(t)$ to coast the brew water temperature down a controlled $6^\circ\text{C}$ ramp—muting late-stage harsh tannins and bitterness.

* **Ascending Profile (**$88^\circ\text{C} \to 94^\circ\text{C}$**):** Pre-cools the upper boiler via a $1.5\text{ s}$ solenoid purge prior to lock-in, then pins $u_{\text{SSR}} = 1.0$ throughout pre-infusion and ramp.

## 4. Non-Linear Fluid Dynamics & Hydraulic Compliance Engine

### 4.1 Convex Pump Surface & Coil Thermal Fade Model

An Ulka EX5/EP5 vibration pump driven by an AC phase-angle dimmer exhibits a convex Pressure-Flow relationship. At control tick $k$, given filtered transducer pressure $P$ ($\text{bar}$), dimmer duty $D \in [0, 1]$, and coil temperature scalar $\alpha_{\text{coil}}$:

$$
Q_{\text{raw}}(P, D, E_{\text{coil}}) = \alpha_{\text{coil}}(E_{\text{coil}}) \cdot Q_{\text{max}}(D) \cdot \left[1 - \left(\frac{\min\left(P,\; P_{\text{stall}}'(D)\right)}{P_{\text{stall}}'(D)}\right)^{\kappa(D)}\right]
$$

Where:

* $Q_{\text{max}}(D) = a_1 D^2 + a_2 D + a_3$ is the open-circuit flow polynomial ($\text{ml/s}$ at $P = 0$).

* $P_{\text{stall}}'(D) = \alpha_{\text{coil}}(E_{\text{coil}}) \cdot \left(b_1 D^2 + b_2 D + b_3\right)$ is the temperature-adjusted deadhead stall pressure ($\text{bar}$ at $Q = 0$).

* $\kappa(D) \in [1.25, 1.65]$ is the empirical convexity exponent calibrated in Module 5.1.

* **Coil Thermal Fade Accumulator (**$E_{\text{coil}}$**):**
  

  $$
  \frac{dE_{\text{coil}}}{dt} = D^2 - \frac{E_{\text{coil}}}{\tau_{\text{cool}}}, \quad \alpha_{\text{coil}}(E_{\text{coil}}) = 1.0 - \beta_{\text{fade}} \cdot E_{\text{coil}}
  $$

  
  With $\tau_{\text{cool}} \approx 95\text{ s}$ and $\beta_{\text{fade}} \approx 0.0028\text{ s}^{-1}$, preventing flow overestimation during long blooming shots or back-to-back pulls.

### 4.2 Dynamic Hydraulic Compliance Observer

Before water exits the coffee puck, pumped water must compress trapped air pockets in the boiler crown, expand the PTFE/braided lines, and compress the headspace inside the basket. If uncorrected, the volumetric integrator thinks water is flowing through the coffee puck during pressure ramp-up when zero liquid is actually moving through the bed.

True puck flow $Q_{\text{puck}}(t)$ ($\text{ml/s}$) is separated from raw pump discharge $Q_{\text{raw}}(t)$ using the pressure derivative $\frac{dP}{dt}$:

$$
Q_{\text{puck}}(t) = \text{clamp}\left(Q_{\text{raw}}(t) - C_{\text{sys}}(P) \cdot \frac{dP}{dt},\; 0.0,\; Q_{\text{max}}(1.0)\right)
$$

Where hydraulic compliance $C_{\text{sys}}(P)$ ($\text{ml/bar}$) is modeled as a two-regime function capturing low-pressure air compression and high-pressure hose elasticity:

$$
C_{\text{sys}}(P) = \frac{V_{\text{air},0}}{(P + 1.0)^2} + c_{\text{hose}}
$$


This equation is automatically parameterized during the Blind-Basket Calibration Sweep.

## 5. Zero-Expertise Automated Calibration & Tuning Wizards

To eliminate manual trial-and-error tuning, two automated calibration routines are exposed in the Web UI and Display UI.

### 5.1 Wizard 1: Automated Hydraulic & Compliance Calibrator (3 Minutes)

1. **Stage A: Open-Flow Sweep (**$P = 0\text{ bar}$**)**

   * **Prompt:** *"Remove portafilter, place empty cup on paired BLE/Drip scale, and press Start."*

   * **Execution:** Steps dimmer duty $D$ through $[0.25, 0.40, 0.55, 0.70, 0.85, 1.00]$ for $4\text{ s}$ each. Computes least-squares fit for $Q_{\text{max}}(D)$ from scale mass rate $\frac{dm}{dt}$.

2. **Stage B: Blind-Basket Compliance & Stall Sweep (**$Q_{\text{puck}} = 0$**)**

   * **Prompt:** *"Insert portafilter with blind backflush disk and press Start."*

   * **Execution:**

     * Runs pump at $D = 0.60$ from $0\text{ bar}$ to $10\text{ bar}$ while logging $P(t)$ at $50\text{ Hz}$. Since $Q_{\text{puck}} = 0$, all pumped water goes into system compliance:
       

       $$
       C_{\text{sys}}(P) = \frac{Q_{\text{raw}}(P, 0.60)}{dP/dt}
       $$

       
       Automatically solves for $V_{\text{air},0}$ and $c_{\text{hose}}$, and detects if the mechanical OPV cracks early (flagged if $\frac{dP}{dt} \to 0$ below $10.5\text{ bar}$ at $D = 1.0$).

     * Vents 3-way solenoid, then steps $D$ through discrete levels to record equilibrium stall pressures $P_{\text{stall}}(D)$.

3. **Stage C: Closed-Loop Shot Auto-Refinement (Recursive Least Squares)**

   * During normal daily espresso pulls whenever a scale is connected, once steady-state pouring begins ($\frac{dm}{dt} > 0.5\text{ g/s}$ and $\vert{}\frac{dP}{dt}\vert{} < 0.2\text{ bar/s}$), the firmware compares estimated $Q_{\text{puck}}(t)$ against actual scale flow $\frac{dm}{dt}(t - \tau_{\text{drip}})$ and applies an exponentially weighted update ($\lambda = 0.05$) to fine-tune the convexity exponent $\kappa(D)$ in NVS.

### 5.2 Wizard 2: Automated Relay-Feedback Thermal Autotuner (8 Minutes)

1. **Stage A: Åström-Hägglund Limit-Cycle Relay Test**

   * Starting from a full boiler, toggles the SSR with a relay amplitude $d = 0.50$ around $T_{\text{ref}} = 88^\circ\text{C}$ with a hysteresis band $\epsilon = 0.3^\circ\text{C}$ for $3$ full oscillation cycles.

   * Extracts oscillation amplitude $a$, ultimate period $T_u$, and dead time $\tau_d$ (measured from relay switch to inflection point):
     

     $$
     K_u = \frac{4 d}{\pi \sqrt{a^2 - \epsilon^2}}
     $$

   * Computes non-overshooting SIMC (Skogestad Internal Model Control) PI gains tailored for the Smith Predictor loop:
     

     $$
     K_p = \frac{0.5 K_u}{1 + \tau_d / T_u}, \quad \tau_i = \min\left(T_u,\; 4(\tau_c + \tau_d)\right)
     $$

2. **Stage B: Dynamic Cold-Water Disturbance Identification**

   * Runs the pump at $100\%$ open flow for $15\text{ s}$ while applying $u_{\text{ff}}(t)$, measuring any residual temperature deviation to auto-calibrate the heater coupling efficiency scalar $\eta$ and inlet water temperature $T_{\text{in}}$.

## 6. Steam Parity & Superiority: Closed-Loop Flash-Boiler Engine

Although daily workflow focuses on straight black espresso, implementing closed-loop flash-steam brings the machine past Gaggiuino's "Dream Steam" capability.

### 6.1 Thermodynamic Basis of Continuous Steam on a $100\text{ ml}$ Boiler

The latent heat of vaporization of water at $140^\circ\text{C}$ ($3.6\text{ bar}$ absolute / $2.6\text{ bar}$ gauge) is $h_{fg} \approx 2144\text{ kJ/kg}$, plus sensible heat $c_{p,w}(140 - 20) \approx 502\text{ kJ/kg}$ (total enthalpy change $\Delta h \approx 2646\text{ J/g}$).
With $P_{\text{max}} = 1370\text{ W}$ continuously active ($u_{\text{SSR}} = 1.0$), the boiler can continuously vaporize a steady-state water mass influx of:

$$
\dot{m}_{\text{flash,max}} = \frac{1370\text{ W}}{2646\text{ J/g}} \approx 0.518\text{ g/s}
$$

### 6.2 Closed-Loop Pressure-Targeted Steam Injection

Unlike open-loop pulsed steam routines that guess when the steam valve is opened, our firmware uses the $0\text{--}5\text{V}$ pressure transducer and boiler thermistor in a synchronized state machine:

1. **Pre-Purge & Superheat:** Heating element locks at $100\%$ duty until $T_b = 148^\circ\text{C}$ ($P \approx 3.5\text{ bar}$ gauge).

2. **Valve-Open Detection:** The user opens the steam knob. The pressure transducer immediately detects a negative pressure derivative ($\frac{dP}{dt} < -0.15\text{ bar/s}$).

3. **Closed-Loop Flash Injection:** A dedicated PI controller modulates Ulka pump phase-angle pulses ($2\text{ Hz}$ burst frequency) to maintain boiler pressure at a constant $P_{\text{steam,set}} = 2.2\text{ bar}$, bounded by the thermodynamic ceiling $Q_{\text{raw}} \le 0.52\text{ ml/s}$ so the boiler never floods or quenches.

## 7. "Beyond Gaggiuino": Next-Gen Extraction Features

### 7.1 Real-Time Darcy Puck Resistance ($R_{\text{puck}}$) & "Auto-Save" Channeling Guard

By Darcy's Law, turbulent-transitional flow through a compressed coffee bed obeys:

$$
R_{\text{puck}}(t) = \frac{P(t)}{Q_{\text{puck}}(t)^2 + \epsilon_0} \quad \left[\text{bar}\cdot\text{s}^2/\text{ml}^2\right]
$$


During a healthy extraction, $R_{\text{puck}}(t)$ peaks after puck compression and declines gradually as solubles dissolve ($\frac{d\ln R_{\text{puck}}}{dt} \approx -0.02\text{ to }-0.05\text{ s}^{-1}$).

* **Auto-Save Trigger:** If $\frac{d\ln R_{\text{puck}}}{dt} < -0.18\text{ s}^{-1}$ during high-pressure extraction (indicating a sudden structural blowout or channel forming in the puck), the state machine overrides the pressure target and switches instantaneously to a **Protective Flow-Capped Taper** ($Q_{\text{target}} = 1.8\text{ ml/s}$), dropping pressure automatically to prevent high-velocity channeling from ruining the cup.

### 7.2 Acoustic & Half-Wave Pre-Infusion Shaping

Standard phase-angle dimming at low duty cycles ($15\text{--}30\%$) produces a harsh $50\text{ Hz}$ mechanical buzz from the Ulka solenoid. During low-pressure pre-infusion ($P < 2.5\text{ bar}$), the firmware switches the pump driver from pure phase-angle cutting to **Bresenham Integral Half-Wave Skipping** combined with soft phase-angle ramps (e.g., firing 1 soft pulse every 4 mains cycles at $12.5\text{ Hz}$), producing silent, gentle water delivery onto the dry puck.

### 7.3 Post-Shot Telemetry & Automated Grind Advisor

At the end of every shot, the firmware evaluates the time-to-peak-resistance ($t_{\text{compress}}$) and mean extraction resistance $\bar{R}_{\text{puck}}$ against the active profile's ideal window and publishes an actionable recommendation via MQTT / Web UI / Display:

$$
\Delta_{\text{grind}} = K_{\text{grind}} \cdot \ln\left(\frac{\bar{R}_{\text{puck,actual}}}{\bar{R}_{\text{puck,target}}}\right)
$$


*(e.g., "Shot ran fast (*$\bar{R} = 1.1\text{ vs }1.8$*): Adjust grinder* $\approx 1.5\text{ steps}$ *finer next shot.")*

## 8. Wireless Battery-Powered Magnetic Display Remote

### 8.1 Architectural Advantage of the GaggiMate Display

Unlike Gaggiuino (where the screen is hardwired to the main STM32 controller), GaggiMate uses a **two-MCU architecture**:

1. **Internal Controller PCB (ESP32-S3):** Houses the SSR driver, AC dimmer, pressure ADC, and thermistor circuitry inside the Gaggia chassis. Runs all real-time control loops, Web UI, and Home Assistant MQTT.

2. **Display Node (LilyGo T-RGB):** An independent ESP32-S3R8 board with a $2.1\text{''}$ $480 \times 480$ round capacitive touchscreen that communicates with the internal PCB **wirelessly over BLE**. In a stock install, the 2-wire harness to the display carries only $5\text{V}$ and $\text{GND}$.

Furthermore, the LilyGo T-RGB PCB already includes a **built-in battery charging IC and a** $1.25\text{ mm}$ **JST (Molex PicoBlade) LiPo connector** on the back of the board.

### 8.2 Hardware Bill of Materials (Wireless Display Mod)

* **Display Board:** Stock GaggiMate LilyGo T-RGB (ESP32-S3R8).

* **Battery Cell:** $3.7\text{ V}$ $1200\text{ mAh}$ LiPo pouch cell ($603450$ form factor) or high-temperature $3.2\text{ V}$ LiFePO4 cell with $1.25\text{ mm}$ 2-pin JST plug.

* **Magnetic Mounting:** $4 \times$ N52 Neodymium magnets ($8\text{ mm} \times 3\text{ mm}$) recessed into the rear housing with a $0.8\text{ mm}$ silicone friction pad to prevent sliding or scratching the brushed stainless steel chassis.

* **Enclosure Material:** 3D-printed in **ASA, ABS, or Polycarbonate (PC)** (never PLA, which deforms at $55^\circ\text{C}$).

### 8.3 Thermal Isolation & Battery Safety Design

The top and upper-rear chassis panels of the Gaggia Classic Pro reach $50^\circ\text{C}\text{ to }62^\circ\text{C}$ when idle. Pressing a LiPo pouch cell directly against a $60^\circ\text{C}$ stainless steel wall causes rapid electrolyte degradation and swelling.

* **Enclosure Air-Gap Standoff:** The 3D-printed rear backplate incorporates a $5.0\text{ mm}$ **ventilated stand-off ring** and an internal $2.0\text{ mm}$ closed-cell silicone foam thermal barrier between the magnetic mounting plane and the battery bay, keeping cell temperature below $35^\circ\text{C}$ even when magnetically docked to the back or side of the machine.

* **Countertop Puck Mode:** Because the magnetic backplate is flat and weighted, the display can sit on the counter next to the grinder as a wireless remote puck, stick magnetically to the side/back of the Gaggia, or snap onto a metallic fridge/coffee bar backsplash.

### 8.4 Display Firmware Power Management State Machine

At full backlight and active BLE polling, the LilyGo T-RGB draws $\approx 190\text{ mA}$ (draining a $1200\text{ mAh}$ battery in $\approx 6\text{ hours}$). Adding a 3-state power manager to the `display/` firmware target extends battery life to **2–3 weeks per USB-C charge**:

1. **Active Brew/Interactive Mode (**$190\text{ mA}$**):** Full $60\text{ Hz}$ LVGL UI rendering and $10\text{ Hz}$ telemetry updates while the machine is actively heating or pulling a shot.

2. **Dimmed Idle Mode (**$45\text{ mA}$**):** Triggered after $45\text{ s}$ of inactivity while the machine is on; reduces backlight PWM to $15\%$ and lowers CPU clock from $240\text{ MHz}$ to $80\text{ MHz}$.

3. **Deep Sleep Mode (**$< 1.2\text{ mA}$**):** Triggered automatically when the internal GaggiMate PCB enters Standby/Off mode (or after $5\text{ minutes}$ of idle). Shuts down the ST7701S RGB panel via the XL9535 I2C IO expander and configures the FT3267 capacitive touch interrupt pin (`TP_INT`) as an `esp_sleep_enable_ext0_wakeup` source. Touching the glass wakes the remote in $< 350\text{ ms}$.

## 9. AI-Assisted (Claude) Development & Staged Implementation Playbook

### 9.1 Architectural Decoupling for Safe LLM Coding

To prevent Claude from introducing hardware-level race conditions or breaking stock GaggiMate WebSocket/BLE stacks, all custom control algorithms are isolated in a pure C++20 library with zero ESP-IDF/Arduino dependencies:

```
gaggimate/
├── lib/
│   └── control_physics/          # <-- Pure C++20 (100% testable on PC/Mac)
│       ├── ThermalObserver.hpp   # 2-Node state observer, Smith predictor & Feed-Forward
│       ├── PumpSurface.hpp       # Convex P-Q-Duty surface, coil fade & RLS adaptor
│       ├── ComplianceModel.hpp   # Dynamic dP/dt volume compensation & Darcy Rpuck
│       ├── FlashSteamEngine.hpp  # Closed-loop pressure-targeted steam pulse controller
│       └── AutoCalibrator.hpp    # State machines for Hydraulic & Relay Thermal wizards
├── test/
│   └── test_native_physics/      # <-- Desktop Software-in-the-Loop (SIL) Plant Simulator
│       ├── VirtualGaggiaPlant.hpp# ODE solver simulating boiler, group, Ulka pump & puck
│       └── test_controllers.cpp  # Automated assertions run via `pio test -e native`
└── src/
    └── controller/               # Thin adapter hooking `control_physics` into 50Hz ISR

```

### 9.2 Software-in-the-Loop (SIL) Verification Workflow

Before flashing any Claude-generated code to your RI9480/14, Claude runs `pio test -e native` on your computer against `VirtualGaggiaPlant.hpp`, which simulates:

* Thermal dead time ($\tau_d = 3.2\text{ s}$), cold water influx, and group head conduction.

* Non-linear pump compression, air headspace compliance, and simulated puck channeling events.

* Safety invariants (asserting that $u_{\text{SSR}} \equiv 0$ whenever $T_{\text{sens}} > 155^\circ\text{C}$ or sensor reads open/short circuit).

### 9.3 Staged Rollout Roadmap

* **Phase 0 (Day 1 - Hardware & Stock Baseline):**

  * Install GaggiMate Pro kit on RI9480/14, retain the factory $12\text{--}15\text{ bar}$ OPV spring, bridge the EU Eco-Timer PCB (`C.1` to `C.2`), and flash stock GaggiMate Pro firmware.

  * Pair BLE scale and verify Home Assistant MQTT entities and basic Pro profiles.

* **Phase 1 (Wireless Magnetic Display Mod):**

  * Print the thermally isolated magnetic T-RGB enclosure in ASA, plug in the $1200\text{ mAh}$ battery via the $1.25\text{ mm}$ JST header, and flash the deep-sleep wake-on-touch display patch.

* **Phase 2 (Thermodynamic Engine & Auto-Tuner):**

  * Implement `ThermalObserver.hpp` and `AutoCalibrator.hpp` (Relay-Feedback Tuner). Verify in SIL simulation, flash to controller, and run the 8-minute Thermal Wizard. Enjoy zero-droop intra-shot temperature stability and dynamic group-warmup compensation.

* **Phase 3 (Non-Linear Pump, Compliance & Auto-Save Flow Control):**

  * Implement `PumpSurface.hpp` and `ComplianceModel.hpp`. Run the 3-minute Open-Flow + Blind-Basket Wizard. Enable real-time Darcy $R_{\text{puck}}$ channeling protection and closed-loop flow profiling.

* **Phase 4 (Closed-Loop Flash Steam & Internal HX711 Drip Scale):**

  * Enable `FlashSteamEngine.hpp` for continuous pressure-locked steaming, and optionally wire dual half-bridge load cells under the drip tray to the spare GaggiMate Pro PCB expansion GPIO pins for zero-latency gravimetric auto-stop.