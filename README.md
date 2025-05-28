# Robotic Arm + Chassis System  
Senior Design Project – Drexel University – 2025  
Gill Saligman — jsaligman@gmail.com • www.linkedin.com/in/gill-saligman-3a81a7255

This project is a modular robotic arm mounted to a custom motorized chassis, designed for autonomous surface exploration. It was built for the NASA RASC-AL competition and represents a full-stack engineering effort — I developed everything from the mechanical structure to the embedded firmware to the wireless communication system.

---

## Overview

- 4-wheeled custom chassis with differential drive  
- 7-DOF robotic arm with closed-loop PID control  
- Onboard Raspberry Pi 4 running a Wi-Fi hotspot and MQTT broker  
- Real-time wireless control using Xbox controller or SSH  
- Supports both position mode (PID to angle) and velocity mode (motor speed)

Each joint of the arm is self-contained — powered and controlled by just four wires (high-voltage, low-voltage). Inside each module is a motor, driver, potentiometer, and microcontroller, making the system scalable and modular.

---

## Mechanical Design

- Frame built from 1" x 1" aluminum extrusion  
- All joints and brackets are custom 3D-printed  
- Uses custom harmonic drives and gear reductions for torque and control  
- Designed for high modularity and minimal weight  
- Arm was simulated and tested for realistic lifting force and torque demands

---

## Electrical Architecture

- Power:  
  - 24V input for motors  
  - 5V buck converters for Pi and microcontrollers  
- Custom joint PCBs:  
  - ESP8266 (D1 Mini)  
  - DRV8871 motor driver  
  - Potentiometer for angle feedback  
- Wiring between joints is minimal: shared power and logic lines only

---

## Software & Communication

- MQTT broker (Mosquitto) runs on the Pi and handles all messaging  
- Joint commands are sent in this format:  
  [DL, DR, J3, J4, J5, J6, G]  
  - DL/DR: left/right chassis motor speeds (velocity mode)  
  - J3–J6: joint control as either Pxx.xx or Vxx.xx  
  - G: gripper angle (0–180 degrees)

Each ESP8266 joint listens for its own command, figures out if it's a position or velocity instruction, and runs either a PID loop or a direct speed output. Every joint sends its current state back to the Pi for live feedback.

---

## Control Options

- SSH terminal for sending manual commands  
- Xbox controller support with mode toggles, deadzones, and analog control  
- Commands are published via MQTT and received by joints in real time

---

## Live Feedback

Each joint sends back its real-time position continuously:

🟢 DESIRED : ['0.00', '0.00', 'P30.00', 'V-0.50', 'X', 'X', '60']  
🔵 ACTUAL  : ['X', 'X', '29.80', '-86.12', 'X', 'X', '60']

---

## Skills Used

Mechanical Engineering  
- Designed and simulated load-bearing robotic joints  
- Built custom harmonic drive gearboxes  
- Optimized structures for modularity, weight, and performance  
- CAD modeling in Fusion360, full FDM 3D printing workflow  

Electrical Engineering  
- Designed and wired low-voltage and high-voltage systems  
- Built and tested custom PCBs for joints  
- Integrated potentiometers for analog angle feedback  

Embedded Systems  
- Wrote MicroPython firmware for ESP8266 boards  
- Tuned PID loops for accurate motion control  
- Handled noisy analog data and real-time motor response  

Networking & Communication  
- Set up Wi-Fi access point and MQTT broker on Raspberry Pi  
- Designed distributed control architecture over MQTT  
- Ensured robust communication with watchdogs and timeouts  

Software  
- Python scripting for input handling and wireless control  
- Implemented Xbox controller logic and feedback  
- Built a flexible command parser with mode switching, clamping, and failsafes

---

Let me know if you'd like a demo video or hardware walkthrough!
