# Smart Boiler Control with Node-RED

This project implements an intelligent, event-driven PID controller for an instantaneous water heater with low thermal mass. It features robust signal filtering, a hardware-aware standby guard, anti-windup clamping, and an online self-learning system identification routine.

## System Architecture
The control system operates fully event-driven. The temperature sensor transmits data via MQTT to the broker only when an actual value change occurs. The Node-RED Function Node reacts immediately to these discrete events.

[ Temperature Sensor ] ---> [ Node-RED Function Node ] ---> [ PWM Actuator ](PID + Learning)

---

## Core Concepts & Implementations

### 1. Event-Driven Main Control Loop (`onBoilerTempIn`)
The calculation loop is triggered exclusively by incoming temperature update events. To isolate the quantization step noise of digital thermometers (e.g., DS18B20), the derivative term calculates the rate of change directly from the physical temperature gradient instead of raw error deltas.
* **Proportional (P):** Reacts instantly to temperature errors ($e = T_{set} - T_{actual}$).
* **Integral (I):** Corrects steady-state errors over time.
* **Derivative (D):** Stabilizes response using the inverted smoothed physical gradient ($-K_d \cdot \frac{dT}{dt}$).

### 2. Indirect Flow Detection & Standby Guard
Since the system lacks a physical flow meter, it infers the state of the water tap implicitly via thermodynamic behavior.
* **Standby Trigger:** If the temperature falls below $T_{set} - 5^\circ\text{C}$ AND the physical gradient is flat or negative ($\frac{dT}{dt} \le 0$), the hardware is assumed to be off due to closed flow.
* **Protection:** The integral state is flushed to zero instantly during standby. This prevents integral wind-up while the tap is closed.
* **Auto-Wakeup:** As soon as the internal flow switch activates the heater, the temperature rises. The gradient becomes positive, exiting standby mode immediately to let the PID controller take over seamlessly.

### 3. Saturation-Based Anti-Windup (Conditional Integration)
Instead of arbitrary boundary clamping, the integrator utilizes conditional integration combined with an actuator feedback check.
* **Logic:** The integration freezes if the calculated ideal output exceeds physical limits ($min\_pwm$ or $max\_pwm$) AND the control error drives the system deeper into saturation.
* **Hysteresis:** A small configurable band ($\pm 0.5\% \text{ PWM}$) prevents continuous toggling of the integration state near boundaries.

### 4. Online Self-Learning System Identification
The node dynamically tunes itself by analyzing the step response during significant PWM adjustments ($\Delta \text{PWM} > 20\%$). It treats the setup as a First-Order Plus Dead-Time (FOPDT) process.

* **Sensor Latency Tracking ($T_t$):** Measures the exact time delay from the PWM jump until the sensor registers a continuous temperature increase. This removes digital sensor delay from physical system data.
* **Filtered Gradient:** A low-pass filter ($\alpha = 0.2$) processes raw temperature increments to calculate an optimal system acceleration rate ($m_{max}$) without quantization noise.
* **Steady-State Detection:** Triggers calculation once the heating rate flattens out ($\text{Gradient} < 0.05^\circ\text{C/s}$).

### 5. Automated PID Tuning (Chien-Hrones-Reswick Method)
Once a learning cycle finishes successfully, the optimal constants are derived using the CHR setpoint regulation equations (20% overshoot criteria):

$$K_p = \frac{0.95 \cdot T}{K_s \cdot T_t}$$
$$T_i = 1.4 \cdot T \implies K_i = \frac{K_p}{T_i}$$
$$T_d = 0.42 \cdot T_t \implies K_d = K_p \cdot T_d$$

*Because $T_t$ resides in the denominator of $K_p$, the controller safely self-throttles if the digital thermometer presents severe conversion latencies.*

---

## Technical Specifications
* **Code Style:** Strictly structured using early returns, explicit sub-statement grouping, no loose semicolons, and secure Yoda-conditions (`0 >= variable`) to eliminate assignment accidents.
* **State Management:** All internal registers (`integral`, `prevError`, `learn`) persist safely inside Node-RED's context memory pool across separate message cycles.