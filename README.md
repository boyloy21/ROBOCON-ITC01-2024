# 🌾 ROBOCON ITC01 — 2024 | ABU Robocon "Harvest Day"

<p align="center">
  <img src="https://img.shields.io/badge/Team-ITC01-darkgreen?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/University-Institute_of_Technology_of_Cambodia-blue?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Competition-ABU_Robocon_2024-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Theme-Harvest_Day-yellow?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/MCU-STM32-blue?style=for-the-badge&logo=stmicroelectronics"/>
  <img src="https://img.shields.io/badge/Language-C-lightgrey?style=for-the-badge&logo=c"/>
</p>

<p align="center">
  <em>Official firmware repository for Team <strong>ITC01</strong> — Institute of Technology of Cambodia<br>
  Competing in the <strong>11th Cambodia Robocon 2024</strong> & <strong>ABU Robocon 2024 "Harvest Day"</strong><br>
</p>

---

## 📋 Table of Contents

- [Competition Overview](#-competition-overview)
- [Game Rules Summary](#-game-rules-summary)
- [Our Robots](#-our-robots)
  - [MR1 — Manual Robot (Semi-Autonomous)](#mr1--manual-robot-semi-autonomous)
  - [MR2 — Autonomous Robot (Fully Autonomous)](#mr2--autonomous-robot-fully-autonomous)
- [Repository Structure](#-repository-structure)
- [System Architecture](#-system-architecture)
- [Hardware Stack](#-hardware-stack)
- [Result Videos](#-result-videos)
- [Getting Started](#-getting-started)
- [Team Members](#-team-members)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🏆 Competition Overview

**ABU Robocon 2024** is the Asia-Pacific Robot Contest organised by the **Asia-Pacific Broadcasting Union (ABU)**. The 2024 edition is hosted in **Quang Ninh, Vietnam**, inspired by Vietnam's iconic terraced rice fields.

| Detail | Info |
|--------|------|
| 🎯 **Theme** | *"Harvest Day"* |
| 📍 **Venue** | Quang Ninh, Vietnam |
| 📅 **Year** | 2024 |
| 🏫 **Our Team** | ITC01 — Institute of Technology of Cambodia (ITC) |
| 🇰🇭 **National Event** | 11th Cambodia Robocon 2024 |
| ⚙️ **MCU Platform** | STM32 (HAL Library) |

> 🌾 *The theme "Harvest Day" reflects the traditional highland practice of terraced rice cultivation in Vietnam — teams must simulate the stages of rice farming: planting seedlings, harvesting paddy rice, and storing grains in silos.*

---

## 📜 Game Rules Summary

The game field is divided into **3 Areas**, each robot performing a unique role in the rice farming cycle.

<p align="center">
  <img src="https://github.com/boyloy21/ROBOCON-ITC01-2024/assets/133625347/18b4a525-6631-478e-ad32-5df222f335cd" alt="ABU Robocon 2024 Game Field — Harvest Day" width="800"/>
  <br>
  <em>Official "Harvest Day" game field layout</em>
</p>

### Task Summary

| Area | Task | Robot | Mode |
|------|------|-------|------|
| Area 1 — Planting Zone | Plant seedlings (PVC pipes) into designated positions | MR1 | Semi-Auto |
| Area 2 — Harvest Zone | Harvest paddy rice and empty grains from the field | MR1 + MR2 | Semi-Auto / Auto |
| Area 3 — Storage Zone | Store harvested paddy rice into silos | MR2 | Fully Auto |

### Key Rules
- ⛔ **No communication** allowed between MR1 and MR2 during the match
- ⏱️ **Match duration:** 3 minutes (KO victory possible)
- 🔴 **Emergency STOP button** (red) required on every robot
- 🔁 Robots may **retry** from designated Retry Zones (referee approval required)
- 📦 Robot shipping box: max **1000 × 1600 × 1400 mm**

---

## 🤖 Our Robots

### MR1 — Manual Robot (Semi-Autonomous)

MR1 is the **manually-driven robot** with assisted automation. It handles the delicate tasks of planting seedlings in Area 1 and initial harvesting in Area 2.

MR1 has been developed in **two drive configurations**:

| Version | Drive System | Status |
|---------|-------------|--------|
| **V1 (Primary)** | 4-Wheel Mecanum Drive | ✅ Main competition build |
| **V2 (Alternate)** | 4-Wheel Omni Drive | 🔄 Alternative tested build |

---

#### MR1 — V1: 4-Wheel Mecanum Drive (Primary)

| Feature | Detail |
|---------|--------|
| Drive System | **4-Wheel Mecanum Drive** |
| Control Mode | Semi-autonomous (manual RC + auto assist) |
| Operating Area | Area 1 (Planting) + Area 2 (Harvesting) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Communication | CAN Bus (inter-board) + RC Receiver |


#### MR1 — V2: 4-Wheel Omni Drive (Alternate)

| Feature | Detail |
|---------|--------|
| Drive System | **4-Wheel Omni Drive** |
| Control Mode | Semi-autonomous (manual RC + auto assist) |
| Operating Area | Area 1 (Planting) + Area 2 (Harvesting) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Communication | CAN Bus (inter-board) + RC Receiver |

---

#### MR1 Mechanical Mechanisms
- **Seedling Gripper** — picks and plants PVC pipe seedlings into racks
- **Omnidirectional Drive** — full 360° movement freedom (Mecanum V1 / Omni V2)
- **Lifting Arm** — raises seedlings to the planting height

---

### MR2 — Autonomous Robot (Fully Autonomous)

MR2 is the **fully autonomous robot** — it navigates, detects balls by color using a **RealSense camera**, catches them, and deposits them into the correct silos without any human input during the match.

#### MR2 Drive System

| Feature | Detail |
|---------|--------|
| Drive System | **4-Wheel Omni Drive** |
| Control Mode | Fully Autonomous |
| Operating Area | Area 2 (Harvesting) + Area 3 (Storage/Silos) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Perception | Intel RealSense Camera (Color + Depth) |
| Navigation | IMU (Gyroscope) + Encoder Odometry |
| Communication | CAN Bus (inter-board) |

#### MR2 Autonomous Controller Versions

MR2 has been tested with **two autonomous control methods**:

| Version | Controller | Status | Description |
|---------|-----------|--------|-------------|
| **V1** | **MPC (Model Predictive Control)** | 🔬 Testing / Research | Predictive trajectory planning for smoother path tracking |
| **V2 (Latest)** | **PID Controller** | ✅ Final / Competition build | Reliable closed-loop speed & heading control |

---

#### MR2 Fully Autonomous Pipeline

MR2 operates through the following perception-to-action pipeline:

```
┌──────────────────────────────────────────────────────────────────┐
│                  MR2 AUTONOMOUS PIPELINE                         │
│                                                                  │
│  ┌─────────────────────┐                                         │
│  │  PERCEPTION         │                                         │
│  │  Intel RealSense    │  → RGB frame + Depth frame              │
│  │  Camera             │                                         │
│  └──────────┬──────────┘                                         │
│             ↓                                                    │
│  ┌─────────────────────┐                                         │
│  │  DETECTION          │  Detect ball presence & classify color  │
│  │  Color Classifier   │                                         │
│  │  ┌────────────────┐ │                                         │
│  │  │ Red  / Blue  ✅│ │  → TRUE  : Valid ball → proceed        │
│  │  │ Purple       ❌│ │  → FALSE : Ignore → continue scanning  │
│  │  └────────────────┘ │                                         │
│  └──────────┬──────────┘                                         │
│             ↓  (if TRUE)                                         │
│  ┌─────────────────────┐                                         │
│  │  MOTION TO BALL     │  Robot navigates toward detected ball   │
│  │  PID / MPC control  │  using depth data for distance estimate │
│  └──────────┬──────────┘                                         │
│             ↓                                                    │
│  ┌─────────────────────┐                                         │
│  │  CATCH BALL         │  Activate collector mechanism           │
│  └──────────┬──────────┘                                         │
│             ↓                                                    │
│  ┌─────────────────────┐                                         │
│  │  NAVIGATE TO SILO   │  Move to target silo position           │
│  │  (by ball color)    │  Red ball → Red silo                    │
│  │                     │  Blue ball → Blue silo                  │
│  └──────────┬──────────┘                                         │
│             ↓                                                    │
│  ┌─────────────────────┐                                         │
│  │  DEPOSIT            │  Release ball into silo                 │
│  └─────────────────────┘                                         │
└──────────────────────────────────────────────────────────────────┘
```
## 📁 Repository Structure

```
ROBOCON-ITC01-2024/
│
├── MR1/                      # Manual Robot — Semi-autonomous
│   ├── ROS2/               
│   │   ├── farmer_launch/
│   │   │   │   ├── farmer_blue.launch.py
│   │   │   │   ├── farmer_red.launch.py
│   │   ├── farmer_robot/
│   │   │   │   ├── Farmer_CAN.py         (CAN communication with STM32)
│   │   │   │   ├── Farmer_CANV2.py
│   │   │   │   ├── Farmer_PID_Blue.py    (Main File to run all process)
│   │   │   │   ├── Farmer_PID_Red.py
│   │   │   │   ├── Farmer_PID_RedV2.py
│   │   │   │   ├── Farmer_PS4.py         (PS4 Remote Control)
│   │   │   │   └── Omni_Farmer.py        (Omni wheel model)
│   │   │   │   └── mecanum_armer.py      (Mecanum wheel model)
│   │   │   │   └── pid_controller.py
│   ├── STM32/               # Primary: 4-Wheel Mecanum Drive
│   │   ├── ESP43-STM32F407/              (Recive data from ESP32 to STM32 Using UART) 
│   │   ├── ESP43-RemoteControl/          (Transmition data ESP32 to STM32)
│   │   ├── Main_Board_Farmer/              (Main Board when robot not using Mini-PC) 
│   │   ├── Main_Board_Farmer_ROS/          (When Robot Using Mini PC and take using with ROS2)
│   │   ├── Motor1_2_Farmer/              (Control DC Motor(1, 2) using with PI control of Motor) 
│   │   ├── Motor3_4_Farmer/          (Control DC Motor(3,4) using with PI control of Motor)
│   │   ├── Motor1_2_Farmer/              (Control DC Motor(1, 2) using with PI control of Motor) 
│   │   ├── Shooter_Farmer/          (Control Mechanicsim for Shooting ball)
│   │   ├── collect_Rice/          (Control Mechanicsim for Collect Rice (air))
│   │   ├── joystick_contrller2/          (stm32 remote control to robot using NRF4l01)
│
├── MR2_Robot/                      # Autonomous Robot — Fully Autonomous
│   ├── ROS2/itc01_ws/src/itc01_mr2
│   │   ├── buffalo_robot/
│   │   │   ├── buffalo_robot/
│   │   │   │   ├── Calculate_position_Red.py         (CAN communication with STM32)
│   │   │   │   ├── EKF.py
│   │   │   │   ├── Omni_PidSim.py    (Main File to run all process)
│   │   │   │   ├── Omni_kinematic.py
│   │   │   │   ├── Omni_model.py
│   │   │   │   ├── Path1.csv         (path trajectory test in CSV)
│   │   │   │   └── Path_gamefield1.csv        (path trajectory real in CSV)
│   │   │   │   └── Path_lab1.csv        (path trajectory test in lab in CSV)
│   │   │   │   └── bezier_path.py      (Generate trajectory using Bezier_path start and end point)
│   │   │   │   └── can_buffalo_bBue.py
│   │   │   │   └── can_buffalo_Red.py
│   │   │   │   └── cubic_spline_planner.py
│   │   │   │   └── ekf.py
│   │   │   │   └── imu_HFI_a9.py     (ROS IMU Read)
│   │   │   │   └── imu_calibrate.py
│   │   │   │   └── mecanum_kinematic.py     (Kinematic of Mecanum Wheel (Inverse and Forward))
│   │   │   │   └── mecanum_pidV2.py         (Base Postion control of Menaum wheel with PID)
│   │   │   │   └── mpc_omni.py         (Base Postion control of Omni wheel with MPC)
│   │   │   │   └── omni_pidV6_Blue.py         (Final code to main run in area blue using PID)
│   │   │   │   └── omni_pidV6_RED.py         (Final code to main run in area red using PID)
│   │   │   ├── launch/
│   │   │   │   └── omni_manual.launch.py         (Testing manual check system)
│   │   │   │   └── omni_mpc.launch.py         (Omni running with MPC Controller)
│   │   │   │   └── omni_pid.launch.py         (Omni running with PID controller)
|
│   │   ├── mr2_realsense/
│   │   │   ├── mr2_realsense/run/detect/train
│   │   │   │   ├── util.py         ()
│   │   │   │   ├── val.py         ()
│   │   │   │   ├── yolov8_rs_blue.py         (Detect blue ball and find postion)
│   │   │   │   ├── yolov8_rs_blue.py                 (Detect red ball and find postion)
|
│   ├── STM32/               # Primary: 4-Wheel Mecanum Drive
│   │   ├── Main_board_Buffalo/              (Main board of Buffalo) 
│   │   ├── Motor1_2_Baffalo/              (Control DC Motor(1, 2) using with PI control of Motor) 
│   │   ├── Motor3_4_Buffalo/          (Control DC Motor(3,4) using with PI control of Motor)
│   │   ├── Sensor/              (Read Sensor 2-Laser, Approximately sensor)
│   │   ├── shooter_buffalo/              (Control shooter Motor ) 
```

---

## 🏗 System Architecture

```
┌────────────────────────────────────────────────────────────┐
│                       MR1 (Manual Robot)                   │
│                                                            │
│  RC Receiver ──→ STM32 Main MCU ──→ CAN Bus               │
│                      │                                     │
│              ┌───────┼────────┐                            │
│              ↓       ↓        ↓       ↓                    │
│           Motor FL  Motor FR  Motor RL  Motor RR           │
│          (Encoder) (Encoder) (Encoder) (Encoder)           │
│           ← bts7960 Drivers / PID Speed Loop →            │
│                                                            │
│    [V1: Mecanum Drive]  |  [V2: 4-Wheel Omni Drive]        │
│          Gripper Servo / Mechanism GPIO                    │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│                   MR2 (Autonomous Robot)                   │
│                                                            │
│  RealSense Camera ──→ Perception Module                    │
│       (Color + Depth)    │                                 │
│                          ↓                                 │
│             Ball Detection & Color Classification          │
│             (Red/Blue ✅ → catch | Purple ❌ → skip)       │
│                          │                                 │
│  IMU (Gyro) ──→ STM32 Main MCU ──→ CAN Bus                │
│  Encoders ──→       │                                      │
│              Navigation State Machine                      │
│              [V1: MPC]  |  [V2: PID — Final]               │
│                      │                                     │
│      ┌───────────────┼───────────────┐                     │
│      ↓               ↓               ↓          ↓          │
│   Motor 1         Motor 2         Motor 3    Motor 4       │
│  (Encoder)       (Encoder)       (Encoder)  (Encoder)      │
│              4-Wheel Omni Drive Base                       │
│                                                            │
│          Collector & Depositor Mechanism                   │
└────────────────────────────────────────────────────────────┘
```

---

## 🔧 Hardware Stack

### Microcontroller

| Component | Specification |
|-----------|---------------|
| MCU | STM32F103c8t6 / STM32F401 |
| Clock | Up to 168 MHz (STM32F407) |
| Flash | 1 MB |
| RAM | 192 KB |
| IDE | STM32CubeIDE + STM32CubeMX |
| Debugger | ST-Link V2 |

### Actuators & Drivers

| Component | Role | Qty |
|-----------|------|-----|
| DC Gear Motor + Encoder | Drive wheels (MR1: 4, MR2: 4) | 8 |
| BTS7960 with STM32F103c8t6 | Motor driver | 4–8 |
| Air solenoid | Gripper & mechanisms  | 1-2|
| Air Driver | Driver to control Air | 2–4 |

### Sensors & Peripherals

| Component | Purpose |
|-----------|---------|
| **Intel RealSense Camera** | Ball detection & color classification for MR2 |
| Bno055 / IMU-ROS| Heading / find yaw angle of robot |
| Rotary Encoder | Calculate position X, Y of robot |
| SICK Laser | Calculate positon robot with Wall | 
| Limit Switches | Mechanism end-stop detection |
| TJA1050 CAN Transceiver | CAN Bus inter-board communication |
| LiPo Battery (3S / 4S) | Main power source |
| DC-DC Buck Converter | 24V → 12V / 5V regulation |

---

## 🎥 Result Videos

Testing videos for **MR1** and **MR2** are documented in the GitHub issue below:

> 📹 **[View MR1 & MR2 Testing Result Videos → Issue #2](https://github.com/boyloy21/ROBOCON-ITC01-2024/issues/2)**

The issue contains:
- 🤖 **MR1** — Mecanum V1 and Omni V2 drive testing footage
- 🤖 **MR2** — Fully autonomous pipeline testing (MPC V1 vs PID V2)
- 🎯 Ball detection & color classification demos (RealSense camera)
- 🏆 Field runs and competition preparation clips

---

## Getting Started

### Prerequisites

- **STM32CubeIDE** (v1.12+)
- **ST-Link V2** programmer
- LiPo battery (3S or 4S, 2200–5000 mAh)
- CAN transceiver (TJA1050 or SN65HVD230)
- Intel RealSense SDK (for MR2 perception module)

### 1. Clone the Repository

```bash
git clone https://github.com/boyloy21/ROBOCON-ITC01-2024.git
cd ROBOCON-ITC01-2024
```

### 2. Open Project in STM32CubeIDE

1. Launch **STM32CubeIDE**
2. `File → Open Projects from File System`
3. Select `MR1_Robot/V1_Mecanum/`, `MR1_Robot/V2_Omni/`, or `MR2_Robot/`
4. Click **Finish**

### 3. Verify Configuration

- Open the `.ioc` file and confirm pin assignments match your hardware
- Check the **CAN bit timing** matches your APB1 clock (default: 1 Mbps @ 42 MHz)
- Confirm **Timer settings** for PWM output match your motor drivers
- For MR2: verify RealSense USB connection and perception module initialisation

### 4. Build and Flash

```
Build:  Ctrl + B  (or click the hammer icon)
Flash:  Run → Debug (F11) or Run → Run (Ctrl+F11)
```

### 5. First Power-On Checklist

- [ ] Verify all motor drivers are connected and powered
- [ ] Confirm CAN bus termination resistors (120Ω) at both ends
- [ ] Test RC controller link with MR1 before enabling motor outputs
- [ ] Verify RealSense camera feed and color detection for MR2
- [ ] Run MR2 navigation in open space before field testing
- [ ] Verify Emergency STOP button is accessible and functional on both robots

---


## 👥 Team Members

**Team ITC01 — Institute of Technology of Cambodia**

<p align="center">
  <img src="https://github.com/boyloy21/ROBOCON-ITC01-2024/assets/133625347/5d0f267c-2f1e-41d7-80c9-47943a025e90" alt="Team ITC01 — Robocon 2024" width="700"/>
  <br>
  <em>Team ITC01 at ABU Robocon 2024 "Harvest Day"</em>
</p>

| Role | Responsibility |
|------|---------------|
| 🧠 Team Leader | Overall coordination, strategy |
| ⚙️ Embedded Software | STM32 firmware, motor control, PID / MPC |
| 🕹 Control System | Autonomous navigation, perception, RC tuning |
| 👁 Computer Vision | RealSense integration, color detection |
| 🔌 Electronics | PCB design, wiring, power system |
| 🔩 Mechanical | Robot frame, mechanisms, CAD design |

> We are students from the **Department of Electrical & Automation Engineering**, Institute of Technology of Cambodia (ITC), Phnom Penh 🇰🇭

---

## 🤝 Contributing

This is an active competition project. Contributions and suggestions are welcome from future ITC teams.

1. Fork the repository
2. Create a branch: `git checkout -b feature/improvement-name`
3. Commit: `git commit -m "Improve: description"`
4. Push: `git push origin feature/improvement-name`
5. Open a **Pull Request**
---

## 📚 References

- [ABU Robocon 2024 Official Rules — "Harvest Day"](https://www.abu.org.my/abu-robocon-2024/)
- [STM32F407 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32 HAL Driver Documentation](https://www.st.com/en/embedded-software/stm32cubef4.html)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [Intel RealSense SDK Documentation](https://dev.intelrealsense.com/docs)
- [CAN Bus Specification — Bosch](https://www.kvaser.com/can-protocol-tutorial/)
- [Mecanum Wheel Kinematics Reference](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)
- [Model Predictive Control — Overview](https://en.wikipedia.org/wiki/Model_predictive_control)
- [MR1 & MR2 Testing Videos — Issue #2](https://github.com/boyloy21/ROBOCON-ITC01-2024/issues/2)

---

## 📄 License

This project is licensed under the **ITC License** — open for learning, research, and future ITC Robocon teams.

---

<p align="center">
  <strong>🇰🇭 Representing Cambodia — Institute of Technology of Cambodia (ITC)</strong><br>
  <em>ABU Robocon 2024 "Harvest Day" — Quang Ninh, Vietnam</em><br><br>
  ⭐ Star this repo if it helped your team or inspired your build!
</p>
