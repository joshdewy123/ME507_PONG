
# Leaflet Cutting Automation System 

**Team Members:** Gabriella Aclan, Josh DeWeese, Lizzie Dilao, Jason Wong

## 🛠️ Overview

This project automates the leaflet cutting process using a **Raspberry Pi**. The system is designed to pickup a leaflet from a container, place it flat on a plate, and then store it in a container. The Raspberry Pi controls four stepper motors and one linear actuator through I2C communication with motor controllers. The system runs a defined **finite state machine (FSM)** sequence to:

- Pick up a leaflet  
- Place the leaflet flat on a plate
- Store the leaflet in another container

A simple **Tkinter UI** provides control and monitoring of the system. Motors operate through **cooperative multitasking**, where each control task voluntarily yields execution to allow other tasks to run smoothly and in sequence.

> 💡 **Note:** The program can also run on a regular laptop without any hardware for simulation.

When `main.py` is ran, a UI window will pop up. Clicking **Start** will display terminal updates about the system state and motor actions.

---

## ⚙️ Hardware 

- Raspberry Pi 4B
- 4 × Tic Stepper Motor Controllers
- 1 × Jrk G2 Linear Actuator Controller
- 4 NEMA23 Stepper Motors
- Linear Actuator
- 36V and 12V power supplies
- I2C bus breakout board or custom wiring
- Tkinter-compatible display
- Appropriate wiring for I2C and power

---

## 📁 File Breakdown

| File                  | Description |
|-----------------------|-------------|
| `main.py`             | Launches the full system and FSM loop |
| `task_controls.py`    | FSM logic for leaflet handling sequence |
| `tic_driver.py`       | Low-level I2C interface for Tic stepper controllers |
| `jrk_driver.py`       | Low-level I2C interface for Jrk actuator controller |
| `controls.py`         | Medium-level logic for movement validation |
| `supervisor_UI_task.py` | Tkinter-based user interface code |
| `task_share.py`       | Manages shared memory between tasks |

---

## ▶️ How to Run

1. **Connect and power hardware (if using real devices).**
2. **Enable I2C on Raspberry Pi:**
   ```bash
   sudo raspi-config
   # Go to Interfacing Options → I2C → Enable
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Choose your run mode:**

### 🧰 OPTION A: Run with Real Hardware
In `main.py`, use real I2C devices:
```python
tic11 = TicI2C(bus, 0x0E)
jrk = JrkI2C(bus, 0x12)
```

For unused motors:
```python
tic14 = DummyMotor()
jrk = DummyActuator()
```

### 💻 OPTION B: Run WITHOUT Hardware
Use only dummy instances:
```python
tic14 = DummyMotor()
tic15 = DummyMotor()
tic16 = DummyMotor()
tic17 = DummyMotor()
jrk = DummyActuator()
```

5. **Run the program:**
   ```bash
   python3 main.py
   ```

---

## 🖥️ User Interface Overview

The UI is built using **Tkinter** and allows live monitoring and control of the automation process.

### Key Features

- **START**: Launches the automation sequence.
- **STOP**: Halts all motors immediately.
- **APPLY SETUP**: Confirms initial configuration before starting.
- **Status Display**: Shows current FSM state like _"Picking Up Leaflet"_.

> **Tips:**
> - Always click **Apply Setup** before clicking **Start**
> - Dummy motors allow full simulation and UI use without hardware

---

## 📝 Notes
- Physical Pins 15 and 16 are SDA6 and SCL6
- Bus 6 on I2C was used because the designated I2C pins are known for bugs 
- Fake motors print extra debug info when used on laptops.
- `[Errno 121] Remote I/O error` means a device is disconnected or miswired.
- Edit motor timing/behavior in `task_controls.py`
- The UI **requires a display**; it will not show over SSH/headless mode.
- Install Tkinter if missing:
  ```bash
  sudo apt-get install python3-tk
  ```
- UI logic is in: `supervisor_UI_task.py`
- If the **Start** button fails after Apply Setup, restart the program.
- Resume after Pause/Terminate is not yet implemented.

---

## 📞 Support

Refer to comments in each `.py` file for inline guidance.  
For further support, contact the developer directly.

---
