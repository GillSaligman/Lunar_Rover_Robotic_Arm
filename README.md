# Robotic Arm + Chassis System  
**Senior Design Project • Drexel University • 2025**  
_Gill Saligman — [jsaligman@gmail.com](mailto:jsaligman@gmail.com) • [LinkedIn](https://www.linkedin.com/in/gill-saligman-3a81a7255)_

This project is a modular robotic manipulator mounted to a motorized chassis, designed for planetary surface operations and prototyped for NASA's RASC-AL competition. I developed the full system architecture — mechanical, electrical, and software — to demonstrate autonomy, communication, and precision motor control across distributed subsystems.

---

## 🔧 Overview

- **Platform**: Custom 4-wheeled rover chassis  
- **Arm**: 7-DOF modular robotic arm with PID control  
- **Brain**: Raspberry Pi 4 (onboard) hosting Wi-Fi and MQTT broker  
- **Communication**: MQTT over local Wi-Fi — robust, scalable, and fast  
- **Control Modes**:  
  - **Position mode**: PID loop to target angle using potentiometer feedback  
  - **Velocity mode**: Direct motor control with acceleration handling  
- **Control Interfaces**: SSH and Xbox controller via MQTT

Each arm joint is modular — receiving only **four wires** (HV, LV) — and contains its own motor controller, ESP8266 microcontroller, and potentiometer.

---

## 🛠 Mechanical Design

- Structural frame: 1"x1" aluminum extrusion  
- All joints, brackets, and harmonic drives: **3D printed** and custom-designed  
- Arm optimized for modularity, weight, and reach/mobility  
- Joint torque calculated based on load case simulations and lifting specs  
- Gear reductions designed to meet torque and inertia requirements  

---

## ⚡ Electrical Architecture

- **Power**:  
  - 24V input from wall supply  
  - 5V buck regulator powers Raspberry Pi and logic lines  
- **Joint Boards** (Custom PCBs):  
  - ESP8266 (D1 Mini) microcontroller  
  - DRV8871 motor driver  
  - Potentiometer for joint angle feedback  
  - PWM/MQTT-compatible firmware  
- **Minimal wiring**: Only 24V and 5V lines between joints

---

## 💻 Software & Communication

- **MQTT Broker**: Hosted on Raspberry Pi (Mosquitto)  
- **Desired Values** sent in bracket format:  
  ```[DL, DR, J3, J4, J5, J6, G]```  
  - `DL/DR` = Drive train left/right velocity  
  - `J3–J6` = `P45.0` (position) or `V-0.6` (velocity)  
  - `G` = Gripper angle (0–180)

- **ESP8266 Joint Firmware**:  
  - Parses desired angle from bracket  
  - Switches mode (P/V) dynamically  
  - Runs local PID loop or velocity output  
  - Publishes actual angle back over MQTT

- **Control Options**:
  - SSH terminal (send manual commands)  
  - Xbox controller (via MQTT with mode toggles, LED indicators, deadzones, and analog scaling)

---

## 📡 Live Feedback

Each joint continuously reports actual position in real time:

```plaintext
🟢 DESIRED : ['0.00', '0.00', 'P30.00', 'V-0.50', 'X', 'X', '60']
🔵 ACTUAL  : ['X', 'X', '29.80', '-86.12', 'X', 'X', '60']
```

## 🧠 Skills Used

### Mechanical Engineering
- Torque calculation, load analysis, and harmonic drive design  
- Structural optimization and modular arm design  
- CAD modeling and 3D printing for rapid prototyping  

### Electrical Engineering
- Custom PCB layout and testing  
- Power distribution (24V/5V), buck conversion, and wiring  
- Analog signal handling for potentiometers and motor control  

### Embedded Systems & Firmware
- MicroPython development on ESP8266  
- Closed-loop PID implementation for joint control  
- Real-time sensor feedback and actuator output  

### Networking & Communication
- MQTT protocol using Mosquitto broker on Raspberry Pi  
- Distributed control architecture for modular joints  
- Wi-Fi-based low-latency control with timeout safety  

### Software Development
- Python scripting and MQTT-based communication  
- Xbox controller input parsing and velocity mapping  
- Real-time command routing, deadzone handling, and telemetry logging
