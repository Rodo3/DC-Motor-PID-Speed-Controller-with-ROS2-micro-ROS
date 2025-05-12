# 🚀 DC Motor PID Speed Controller with ROS2 & micro-ROS

This project presents the **design, simulation, and implementation of a real-time PID controller** for a brushed DC motor using an ESP32 microcontroller running **micro-ROS**, a lightweight ROS 2 client library tailored for resource-constrained embedded systems. The system achieves precise speed control, with a fully reconfigurable control interface through ROS 2 topics, and is validated via both simulations and physical experiments.

---

## 📌 Project Summary
- **Objective:** Precisely control the angular velocity of a DC motor using a PID algorithm and monitor system performance through ROS 2 tools.
- **Platform:** Embedded ESP32 node with micro-ROS communicating over serial with a ROS 2 host system.
- **Control Strategy:** Proportional–Integral–Derivative (PID) control with dynamic tuning of Kp, Ki, and Kd.
- **Input Signals:** Step, square, and sine wave reference profiles generated from a separate ROS 2 signal generator node.
- **Sampling Time:** 35 ms, determined based on motor response characteristics and Nyquist sampling criteria.

---

## 🧠 Features
- **Dynamic PID Tuning:** Control gains can be updated in real time via dedicated ROS 2 topics (`/Kp`, `/Ki`, `/Kd`).
- **Fully Decentralized Control:** All core control computations occur on the ESP32, enabling scalable deployment without reliance on high-performance central systems.
- **Encoder-Based Feedback:** Motor speed is calculated from encoder pulses and published on the topic `/motor_output`.
- **Dead-Zone Compensation:** Ensures motor behavior remains stable even in low PWM ranges by adjusting signal thresholds.
- **Anti-Windup Strategy:** Prevents the integral term from growing unbounded when actuators reach saturation.
- **Simulation + Experimentation:** System models were implemented in both ROS 2 and MATLAB to validate behavior under realistic and ideal conditions.

---

## 🧰 Technologies Used
- **Programming Languages:** C++ (Arduino/micro-ROS), Python (ROS 2 nodes)
- **Frameworks:**
  - ROS 2 Foxy Fitzroy
  - micro-ROS
  - rcl/rclc/rclc_executor libraries for embedded ROS 2 nodes
- **Visualization Tools:**
  - rqt_graph (node topology)
  - PlotJuggler (real-time signal monitoring)
- **Simulation Tools:** MATLAB for second-order system dynamics, ROS 2 nodes for real-time modeling
- **Hardware:**
  - ESP32 MCU
  - Brushed DC motor with encoder
  - H-Bridge Driver (L298N)
  - 7.2V 1500mAh LiPo battery

---

## 🧱 System Architecture Overview
### Block Diagram
![Block Diagram](docs/block_diagram.png)

### Key Components
- **ESP32 Node (`motor_node`)**: Reads encoder feedback, performs PID control, generates PWM + direction signals, publishes measured speed.
- **External PC (ROS 2 host)**: Publishes desired setpoint and PID parameters. Also visualizes system data and logs output.
- **ROS 2 Communication:** 
  - Topics: `/set_point`, `/Kp`, `/Ki`, `/Kd`, `/motor_output`
  - Messages: `std_msgs/Float32` and `Float64`

### Control Loop Cycle
1. External ROS 2 node sends:
   - Desired speed reference `/set_point`
   - Control parameters `/Kp`, `/Ki`, `/Kd`
2. ESP32:
   - Reads current velocity via encoder
   - Calculates error `e = set_point - measured_velocity`
   - Computes PID control signal `u(t)`
   - Applies anti-windup and normalizes `u(t)` to [-1, 1]
   - Maps `u(t)` to PWM signal and motor direction (forward/reverse)
   - Publishes updated velocity `/motor_output`

---

## 🛰️ ROS 2 Topics Used
| Topic           | Type       | Description                                               |
|----------------|------------|-----------------------------------------------------------|
| `/set_point`   | Float64    | Desired angular velocity in rev/s                        |
| `/Kp` `/Ki` `/Kd` | Float32 | PID controller gains                                     |
| `/motor_output`| Float64    | Actual motor speed measured by encoder                   |

These topics allow **runtime flexibility** and **low-latency control** in embedded robotics applications. The dynamic updates enable iterative PID tuning without reflashing or recompilation.

---

## ⚙️ PID Control Strategy
### PID Algorithm
The ESP32 calculates the control signal every 35 ms using:
```
error = setpoint - measured_velocity
integral += error * dt
integral = clamp(integral, -max_integral, max_integral)
derivative = (error - error_prev) / dt
control_signal = Kp * error + Ki * integral + Kd * derivative
normalized_signal = clamp(control_signal / max_possible_signal, -1, 1)
```

### Sample Time Selection
To ensure proper dynamic response capture, the sampling time was selected using Nyquist’s criterion and desired resolution:

```
Ts = min(1 / (2 * f_max), τ / n)
```
Where:
- `f_max` is the maximum operating frequency of the motor (3 rev/s)
- `τ` is the characteristic system response time (≈ 0.2 s)
- `n` is the number of samples desired per period (e.g., 6 samples)

Thus, `Ts = min(0.167 s, 0.033 s) ≈ 0.033 s`

This guarantees accurate tracking of reference signals without overloading the MCU.

### Dead-Zone Compensation
Prevents undesired behavior when PWM values are too low for the motor to respond.

### Anti-Windup
Limits the integral term to avoid overshooting when actuator is saturated.

---

## 🧪 Methodology Overview
### Control with ROS2 & micro-ROS
- ESP32 node acts as transducer:
  - Sends PWM and direction to the motor
  - Calculates angular velocity from encoder
  - Publishes data on `/motor_output`
- Control node generates control signal `u(k)` within [-1, 1], published to `/motor_input`
- Input generator node publishes signals like step, square, sine to `/set_point`

### Control Strategy & Tuning
- Uses `std_msgs/Float32` and `Float64`
- Parameters adjusted dynamically via YAML config
- Sampling time: **35 ms**
- PID tuning approach:
  - Increase Kp → faster response
  - Increase Kd → reduced oscillation
  - Increase Ki → reduced steady-state error
- Iterative testing ensured robust, adaptable performance

### Team Responsibilities & Validation
- Hardware design & setup: ESP32 + L298N + encoder wiring
- Software implementation: ROS 2 nodes, firmware logic
- Model simulations:
  - First-order system (ROS 2 real-time)
  - Second-order system (MATLAB)
- Experimental testing confirmed simulation accuracy
- Team collaboration optimized quality and efficiency

---

## 📊 Results
### 🔬 Simulations
- **ROS 2 (First-Order Model):** Enabled live tuning and evaluation
- **MATLAB (Second-Order Model):** Included friction and inertia

### 🧪 Experiments
- Final PID tuning: `Kp = 300`, `Ki = 970`, `Kd = 80`
- Metrics:
  - MSE: 0.0023 (rad/s)²
  - IAE: 0.0181 rad/s·s
  - ITAE: 0.0892 rad/s·s²
  - ISE: 0.0104 (rad/s)²·s
  - Settling Time: 1.2 s
  - Overshoot: 5%

---

## 🔍 Observations
- ROS 2 enables **modular, distributed system design**, ideal for robot platforms with multiple actuators and sensors
- micro-ROS bridges the gap between embedded firmware and ROS 2 middleware
- ROS 2 tools like `rqt_graph` and `PlotJuggler` proved essential for debugging and system analysis
- Including friction and energy loss in simulation improved prediction accuracy

---

## 🎯 Future Work
- Upgrade to hardware PWM for smoother control
- Expand system for multi-motor coordination (e.g., differential drive robots)
- Include feedforward control and Kalman filtering for noise reduction
- Integrate safety features such as watchdog timers and fault detection

---
