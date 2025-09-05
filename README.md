# 4ward

# Engineering Materials for WRO Future Engineers 2025

This repository contains the engineering documentation for **UneCirciut's self-driving vehicle**, designed for the **World Robot Olympiad (WRO) Future Engineers 2025** competition.  
Our project showcases a fully autonomous vehicle capable of navigating challenges through innovative **mobility, power management, sensing, and obstacle management strategies**.

---

## 👥 Team Introduction

**Team UneCirciut** consists of three members:  
- **Aysun Abdullayeva**  
- **Zubeyda Hasanli**  
- **Aysu Huseynova**

We are students from **Azerbaijan State University of Economics**, passionate about robotics and autonomous systems.  
Our goal is to design a robust vehicle that balances **performance, reliability, and creativity**, inspired by real-world self-driving technology.

📂 Team photos will be uploaded in the [`t-photos/`](./t-photos/) directory:  
- `team-official.jpg` → Formal team photo  
- `team-fun.jpg` → Fun team photo  

---

## 🚗 Vehicle Design Overview

Our vehicle is designed to excel in the WRO Future Engineers challenges, with a focus on **autonomous navigation, obstacle avoidance, and efficient power usage**.

### 🔧 Mobility
- Drive system powered by a **single 6V 150 RPM DC motor** with an attached wheel.  
- Rear-wheel drive via **gear system**.  
- **Surpass Hobby 9g Digital Servo** controls the front wheels for steering.  
- Motor driver: **TB6612FNG**, interfaced with **Raspberry Pi 4**.  

This configuration allows **sharp turns, stable movement, and smooth acceleration**.

---

### 🔋 Power Management
- Powered by **three 3.7V 18650 Li-ion batteries** in series → ~**11.1V / 2200mAh**.  
- **Step-down voltage regulator (HW-083)** provides 5V for Raspberry Pi and sensors.  
- Includes **Battery Management System (BMS)** to prevent over-discharge.  
- Optimized for **20 minutes of continuous operation**.  

⚡ Challenge: Voltage drops under heavy motor load → **solved with capacitors** for stable supply.  

---

### 📡 Sensing
Our vehicle employs multiple sensors for navigation and obstacle detection:

- **HC-SR04 Ultrasonic Sensors (x3)** → front, left, right (range: 2m).  
- **TCS3200 Color Sensor (x1)** → detect track markers & obstacles (red = right, green = left).  
- **Raspberry Pi Camera Module V1.3** → line-following & visual recognition.  

Data is processed in real time on **Raspberry Pi 4** with noise reduction via **moving average filter**.

---

### 🚧 Obstacle Management
- Vehicle pauses when obstacle detected.  
- Scans with ultrasonic sensors → calculates alternative path using **rule-based algorithm**.  
- Dynamic obstacles:  
  - **Green cube → pass left**  
  - **Red cube → pass right**  

This ensures **reliable navigation** in the WRO environment.  
Sensor noise was reduced by **averaging multiple readings**.

---

## ⚙️ Electromechanical Schematics
Schematics will be uploaded in the [`schemes/`](./schemes/) directory:  

- `wiring-diagram.png` → connections between Raspberry Pi 4, TB6612FNG, HC-SR04, TCS3200, Camera, Servo, and Battery.  

---

## 🖥️ Code Structure and Integration
Source code is located in the [`src/`](./src/) directory, written in **Python** for Raspberry Pi 4.  

### 📂 Code Modules
- `main.py` → Orchestrates main control loop.  
- `sensors.py` → Interfaces with ultrasonic, color sensor, and camera.  
- `motors.py` → Controls DC motor & steering servo via TB6612FNG.  
- `navigation.py` → Implements rule-based pathfinding & line-following.  
- `utils.py` → Helper functions (data filtering, logging).  

### 🔗 Relation to Hardware
- `sensors.py` → GPIO 17,18,27 (ultrasonic), GPIO 22,23 (color sensor), CSI (camera).  
- `motors.py` → GPIO 24,25 (PWM for DC motor), GPIO 12 (servo).  
- `navigation.py` → Processes sensor data to adjust motor speeds.

---

## 🧩 Component Photos
Located in [`component-photos/`](./component-photos/).  

- Raspberry Pi 4 Model B (4GB)  
- Motor + Wheel Set (6V, 150 RPM)  
- TB6612FNG Motor Driver  
- HC-SR04 Ultrasonic Sensor  
- Raspberry Pi Camera Module  
- Servo Motor  
- TCS3200 Color Sensor  

---

## 📸 Photos and 🎥 Videos
- Vehicle Photos → [`v-photos/`](./v-photos/)  
  - `vehicle-front.jpg`, `vehicle-back.jpg`, `vehicle-left.jpg`, `vehicle-right.jpg`, `vehicle-top.jpg`, `vehicle-bottom.jpg`  
- Team Photos → [`t-photos/`](./t-photos/)  
- Demo Videos → [`video/video.md`](./video/video.md) (links to YouTube, ≥30s per challenge)  

---

## 📂 Additional Resources
- [`models/`](./models/) → STL files for 3D-printed chassis parts  
- [`other/`](./other/) → Datasheets for sensors and motors  

---

## 📧 Contact
For questions, contact us at:  
**une.circiut@azseu.edu.az**

---

✅ We encourage other teams to explore this repository for inspiration, in line with the **WRO open-sharing ethos**.
