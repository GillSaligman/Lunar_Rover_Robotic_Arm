## 🚀 Project Overview
**Modular, Wi‑Fi‑driven robotic arm** mounted on a custom rover chassis, built as my Senior Design capstone to prototype a NASA RaSCAL competition rover.  
- **Central Brain:** Raspberry Pi 4 hosts an access‑point + MQTT broker  
- **Drive + Arm Control:** All joints (and drive motors) subscribe to a common 7‑slot “desired” bracket, publish their own “actual” feedback  
- **Modes:** Position (PID‑controlled via potentiometer feedback) or Velocit
