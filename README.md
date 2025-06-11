# ME 507 - Ping Pong Ball Launcher

**Team Members:** Josh DeWeese, Nolan Jeung  
**Support:** Charlie Refvem, Vincent Ngo, Jason Wong

---

## 🛠️ Overview

This project centers around an autonomous ping pong ball launcher designed for two core functionalities:

1. **Ping Pong Practice Mode**: The system scans the edge of a table using LIDAR and targets the shortest distance to simulate a return serve.
2. **Beer Pong Mode**: The robot scans the playing field for red Solo cups using signal intensity and shoots into them.
3. **(Optional)**: Control the launcher via PuTTY or an external controller.

> 💡 Our primary objective was to build a functional robot that could:
> - Use an IMU to calibrate orientation  
> - Use LIDAR for object detection  
> - Use encoders for accurate turning  
> - Use a servo to load balls  
> - Use flywheel motors to launch at specific speeds  

---

## ⚙️ Hardware Summary

- **Main Controller**: STM32F411CEU6 (Blackpill)
- **Actuators**:
  - 2 × Stepper Motors (base orientation)
  - 2 × Flywheel DC Motors (launching)
  - 1 × Servo Motor (ball loading)
- **Sensors**:
  - LIDAR (distance sensing)
  - BNO055 IMU (orientation)
  - 2 × Motor Encoders
- **Power System**:
  - 12V Rail via LM22678 Buck Converter  
  - 3.3V Rail via LMS1587 LDO
- **Custom PCB**: Used to consolidate motor driver circuitry, power rails, and microcontroller

---

## 🧠 Software Implementation

- **Language**: C/C++ using STM32CubeIDE
- **Control Architecture**: Finite State Machine (FSM)
- **Drivers**:
  - Custom motor driver interface
  - I2C drivers for LIDAR and IMU
  - PWM control for servo
- **Coding Style**:
  - FSM-based modular design
  - Lightweight task-based sequencing
  - Code structured for real-time behavior with interrupt-driven UART

---

## 🧪 Sensor Processing & Modeling

- **LIDAR**: Closest distance (Ping Pong Mode) or highest signal intensity (Beer Pong Mode)
- **IMU**: Euler angle readings (heading)
- **Encoders**: Estimate rotation and confirm motor behavior

---

## 🛠️ PCB + System Design Challenges

The team completed a full design cycle:

- Designed and ordered a **custom PCB**
- Sourced and soldered components
- Debugged I2C communication issues
- Validated power rail performance
- Verified motor driver functionality
- Integrated electrical and mechanical subsystems

---

## ❗ Notable Roadblocks & Fixes

| Issue | Fix | Effectiveness |
|-------|-----|---------------|
| ❌ No resistors on IPROPI pins | Soldered 1.5kΩ resistors to breakout board | ✅ Fixed in 1 day |
| ❌ Missing bulk capacitors | Added 330µF caps near motor drivers | ✅ Fixed in 1 day |
| ❌ Missing LIDAR pullups | Added 3.3V pullups on SDA/SCL | ✅ Fixed in 1 day |
| ❌ IMU not responsive | Switched to alternate 3.3V/GND pins | ⚠️ Slower to debug |
| ❌ IMU FSM integration failed | Deferred integration | ⚠️ Future fix |
| ❌ No TIM PWM channel for servo | Later found unused TIM pin | ⚠️ PCB blocked access |

---

## 💡 Lessons Learned

- Always include **bulk caps** near motor drivers.
- **Pull-up resistors** are essential for I2C devices.
- Leave **unused MCU pins accessible** in PCB design.
- Modular code + FSM simplifies **debugging and testing**.
- A working schematic **isn’t always** a working system.

---

## ▶️ How to Run (Demo Instructions)

1. Power the PCB via a 12V barrel jack.
2. Connect STM32 to your PC via ST-Link.
3. Flash firmware using STM32CubeIDE.
4. Connect with PuTTY or any serial terminal.
5. Example commands:
   - `CAL` – calibrate IMU heading *(future implementation)*
   - `FIRE` – launch ping pong ball
   - `MODE` – toggle between game modes

> FSM runs automatically if no command is received.

