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
  Hosted in Quang Ninh, Vietnam 🇻🇳</em>
</p>

---

## 📋 Table of Contents

- [Competition Overview](#-competition-overview)
- [Game Rules Summary](#-game-rules-summary)
- [Our Robots](#-our-robots)
  - [Robot 1 — R1 (Semi-Autonomous)](#robot-1--r1-semi-autonomous)
  - [Robot 2 — R2 (Fully Autonomous)](#robot-2--r2-fully-autonomous)
- [Repository Structure](#-repository-structure)
- [System Architecture](#-system-architecture)
- [Hardware Stack](#-hardware-stack)
- [Software Modules](#-software-modules)
  - [Motor Control](#motor-control)
  - [PID Controller](#pid-controller)
  - [CAN Bus Communication](#can-bus-communication)
  - [IMU & Odometry](#imu--odometry)
  - [Autonomous Navigation](#autonomous-navigation)
  - [Remote Control (RC)](#remote-control-rc)
- [Getting Started](#-getting-started)
- [Pin Configuration](#-pin-configuration)
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

```
┌──────────────────────────────────────────────────────────────┐
│                     GAME FIELD LAYOUT                        │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   AREA 1     │  │   AREA 2     │  │   AREA 3     │      │
│  │  🌱 Planting  │  │ 🌾 Harvesting│  │ 🏚 Storage   │      │
│  │              │  │              │  │              │      │
│  │ Plant seedlings│ Harvest paddy │ Store in silos │      │
│  │  (PVC pipes) │  │    rice      │  │              │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                              │
│  R1 operates in Area 1 & 2        R2 operates in Area 2 & 3 │
└──────────────────────────────────────────────────────────────┘
```

### Task Summary

| Area | Task | Robot | Mode |
|------|------|-------|------|
| Area 1 — Planting Zone | Plant seedlings (PVC pipes) into designated positions | R1 | Semi-Auto |
| Area 2 — Harvest Zone | Harvest paddy rice and empty grains from the field | R1 + R2 | Semi-Auto / Auto |
| Area 3 — Storage Zone | Store harvested paddy rice into silos | R2 | Fully Auto |

### Key Rules
- ⛔ **No communication** allowed between R1 and R2 during the match
- ⏱️ **Match duration:** 3 minutes (KO victory possible)
- 🔴 **Emergency STOP button** (red) required on every robot
- 🔁 Robots may **retry** from designated Retry Zones (referee approval required)
- 📦 Robot shipping box: max **1000 × 1600 × 1400 mm**

---

## 🤖 Our Robots

### Robot 1 — R1 (Semi-Autonomous)

R1 is the **manually-driven robot** with assisted automation. It handles the delicate tasks of planting seedlings in Area 1 and initial harvesting in Area 2.

#### R1 Specifications

| Feature | Detail |
|---------|--------|
| Drive System | **3-Wheel Holonomic (Omni-wheel)** |
| Control Mode | Semi-autonomous (manual RC + auto assist) |
| Operating Area | Area 1 (Planting) + Area 2 (Harvesting) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Communication | CAN Bus (inter-board) + RC Receiver |

#### R1 Mechanical Mechanisms
- **Seedling Gripper** — picks and plants PVC pipe seedlings into racks
- **Holonomic Drive** — 3-wheel omni configuration for full 360° movement freedom
- **Lifting Arm** — raises seedlings to the planting height

#### R1 Drive Kinematics (Holonomic 3-Wheel)

```
           Motor 1 (0°)
              ↑
       ╔══════════════╗
       ║              ║
Motor 3 ←    R1 Body  → Motor 2
(240°)  ║              ║  (120°)
       ╚══════════════╝

Velocity equations:
  Vx = V1·cos(0°)   + V2·cos(120°)  + V3·cos(240°)
  Vy = V1·sin(0°)   + V2·sin(120°)  + V3·sin(240°)
  Wz = (V1 + V2 + V3) / R
```

---

### Robot 2 — R2 (Fully Autonomous)

R2 is the **fully autonomous robot** — it navigates, harvests paddy rice, and stores it in silos without any human input during the match.

#### R2 Specifications

| Feature | Detail |
|---------|--------|
| Drive System | **4-Wheel Mecanum Drive** |
| Control Mode | Fully Autonomous |
| Operating Area | Area 2 (Harvesting) + Area 3 (Storage/Silos) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Navigation | IMU (Gyroscope) + Encoder Odometry |
| Communication | CAN Bus (inter-board) |

#### R2 Mechanical Mechanisms
- **Paddy Rice Collector** — scoops and collects harvested paddy rice
- **Mecanum Drive Base** — 4-wheel mecanum for omnidirectional autonomous navigation
- **Silo Depositor** — transfers collected rice into the correct silo positions

#### R2 Drive Kinematics (4-Wheel Mecanum)

```
      FL Motor ╔═══════╗ FR Motor
        (↗)    ║       ║   (↘)
               ║  R2   ║
        (↘)    ║       ║   (↗)
      RL Motor ╚═══════╝ RR Motor

  Vx  =  (V_FL + V_FR + V_RL + V_RR) / 4
  Vy  =  (-V_FL + V_FR + V_RL - V_RR) / 4
  Wz  =  (-V_FL + V_FR - V_RL + V_RR) / (4 * (Lx + Ly))
```

---

## 📁 Repository Structure

```
ROBOCON-ITC01-2024/
│
├── R1_Robot/                       # Robot 1 — Semi-autonomous holonomic drive
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c              # Main loop & initialization
│   │   │   ├── motor_control.c     # DC motor PWM + direction
│   │   │   ├── pid_controller.c    # PID speed loop
│   │   │   ├── holonomic_drive.c   # 3-wheel omni kinematics
│   │   │   ├── can_comm.c          # CAN bus TX/RX
│   │   │   └── rc_receiver.c       # RC remote input (UART/PWM)
│   │   └── Inc/
│   │       ├── motor_control.h
│   │       ├── pid_controller.h
│   │       ├── holonomic_drive.h
│   │       ├── can_comm.h
│   │       └── rc_receiver.h
│   └── R1_Robot.ioc                # STM32CubeMX config
│
├── R2_Robot/                       # Robot 2 — Fully autonomous mecanum drive
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c              # Main loop & state machine
│   │   │   ├── motor_control.c     # DC motor PWM + direction
│   │   │   ├── pid_controller.c    # PID speed/position loop
│   │   │   ├── mecanum_drive.c     # 4-wheel mecanum kinematics
│   │   │   ├── imu_driver.c        # IMU / gyroscope interface
│   │   │   ├── odometry.c          # Encoder-based position tracking
│   │   │   ├── navigation.c        # Autonomous path execution
│   │   │   └── can_comm.c          # CAN bus TX/RX
│   │   └── Inc/
│   └── R2_Robot.ioc
│
├── Shared/                         # Code shared between R1 and R2
│   ├── pid.c / pid.h
│   ├── can_protocol.h              # CAN message ID definitions
│   └── motor_driver.c / .h
│
└── README.md
```

---

## 🏗 System Architecture

```
┌────────────────────────────────────────────────────────────┐
│                      ROBOT 1 (R1)                          │
│                                                            │
│  RC Receiver ──→ STM32 Main MCU ──→ CAN Bus               │
│                      │                                     │
│              ┌───────┼───────┐                             │
│              ↓       ↓       ↓                             │
│          Motor 1  Motor 2  Motor 3   ← H-Bridge Drivers    │
│         (Encoder)(Encoder)(Encoder)  ← Speed Feedback      │
│              ↑       ↑       ↑                             │
│              └───────┴───────┘                             │
│                   PID Loop                                 │
│                                                            │
│          Gripper Servo / Mechanism GPIO                    │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│                      ROBOT 2 (R2)                          │
│                                                            │
│  IMU (Gyro) ──→ STM32 Main MCU ──→ CAN Bus                │
│  Encoders ──→       │                                      │
│  Sensors  ──→   Navigation &                               │
│               State Machine                                │
│                      │                                     │
│         ┌────────────┼────────────┐                        │
│         ↓            ↓            ↓         ↓             │
│      Motor FL     Motor FR     Motor RL  Motor RR          │
│     (Encoder)   (Encoder)    (Encoder) (Encoder)           │
│         ↑            ↑            ↑         ↑             │
│         └────────────┴────────────┘─────────┘             │
│                    PID Speed Loop                          │
│                                                            │
│          Collector & Depositor Mechanism                   │
└────────────────────────────────────────────────────────────┘
```

---

## 🔧 Hardware Stack

### Microcontroller

| Component | Specification |
|-----------|---------------|
| MCU | STM32F407VGT6 / STM32F401 |
| Clock | Up to 168 MHz (STM32F407) |
| Flash | 1 MB |
| RAM | 192 KB |
| IDE | STM32CubeIDE + STM32CubeMX |
| Debugger | ST-Link V2 |

### Actuators & Drivers

| Component | Role | Qty |
|-----------|------|-----|
| DC Gear Motor + Encoder | Drive wheels (R1: 3, R2: 4) | 7 |
| L298N / BTS7960 H-Bridge | Motor driver | 4–6 |
| Servo Motor (MG996R) | Gripper & mechanisms | 2–4 |
| BLDC Motor + ESC | High-speed mechanism (if applicable) | TBD |

### Sensors & Peripherals

| Component | Purpose |
|-----------|---------|
| MPU6050 / ICM-20602 IMU | Heading / gyroscope for R2 navigation |
| Incremental Encoder (600 PPR) | Motor speed & position feedback |
| TJA1050 CAN Transceiver | CAN Bus inter-board communication |
| FlySky FS-i6X RC Transmitter | Manual control of R1 |
| IBUSS / SBUS Receiver | RC signal decoding |
| Limit Switches | Mechanism end-stop detection |
| LiPo Battery (3S / 4S) | Main power source |
| DC-DC Buck Converter | 24V → 12V / 5V regulation |

---

## 💻 Software Modules

### Motor Control

PWM-based speed control via STM32 Timer, with GPIO direction control.

```c
// motor_control.c — Set motor speed and direction
void Motor_Set(Motor_t *motor, int16_t speed) {
    if (speed >= 0) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
        speed = -speed;
    }
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint16_t)speed);
}
```

---

### PID Controller

Discrete PID implementation for closed-loop speed regulation on each wheel.

```c
// pid_controller.c — Generic PID compute
float PID_Compute(PID_t *pid, float setpoint, float measurement) {
    float error      = setpoint - measurement;
    pid->integral   += error * pid->dt;
    pid->integral    = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);
    float derivative = (error - pid->prev_error) / pid->dt;
    float output     = pid->Kp * error
                     + pid->Ki * pid->integral
                     + pid->Kd * derivative;
    pid->prev_error  = error;
    return CLAMP(output, -pid->output_limit, pid->output_limit);
}
```

#### Recommended Starting Gains

| Robot | Kp | Ki | Kd |
|-------|----|----|----|
| R1 (Omni) | 2.0 | 0.5 | 0.05 |
| R2 (Mecanum) | 2.5 | 0.8 | 0.08 |

---

### CAN Bus Communication

All boards communicate via **CAN 1 Mbps**. Each robot has a defined set of message IDs.

```c
// can_protocol.h — Message ID definitions
#define CAN_ID_R1_SPEED_CMD     0x101   // R1 → Motor Board: speed setpoints
#define CAN_ID_R1_STATUS        0x102   // R1 Motor Board → Main: encoder feedback
#define CAN_ID_R2_SPEED_CMD     0x201   // R2 → Motor Board: speed setpoints
#define CAN_ID_R2_STATUS        0x202   // R2 Motor Board → Main: encoder feedback
#define CAN_ID_R2_NAV_STATE     0x210   // R2 Navigation state broadcast
#define CAN_ID_ESTOP            0x7FF   // Emergency STOP (all nodes)
```

```c
// Send speed commands to motor board
void CAN_SendSpeedCmd(uint16_t id, int16_t v1, int16_t v2, int16_t v3, int16_t v4) {
    uint8_t data[8];
    data[0] = (v1 >> 8) & 0xFF;  data[1] = v1 & 0xFF;
    data[2] = (v2 >> 8) & 0xFF;  data[3] = v2 & 0xFF;
    data[4] = (v3 >> 8) & 0xFF;  data[5] = v3 & 0xFF;
    data[6] = (v4 >> 8) & 0xFF;  data[7] = v4 & 0xFF;
    CAN_SendMessage(id, data, 8);
}
```

---

### IMU & Odometry

Used by **R2** for autonomous heading control and dead-reckoning position tracking.

```c
// imu_driver.c — Read yaw from MPU6050 via I2C
float IMU_GetYaw(void) {
    int16_t raw_gz;
    MPU6050_ReadGyroZ(&raw_gz);
    float gz_dps = raw_gz / 131.0f;               // Convert to °/s
    yaw += gz_dps * SAMPLE_TIME_S;                 // Integrate
    return yaw;
}

// odometry.c — Update robot position from encoder ticks
void Odometry_Update(Odometry_t *odom, int32_t enc_fl, int32_t enc_fr,
                                        int32_t enc_rl, int32_t enc_rr) {
    float vx = (enc_fl + enc_fr + enc_rl + enc_rr) * DIST_PER_TICK / 4.0f;
    float vy = (-enc_fl + enc_fr + enc_rl - enc_rr) * DIST_PER_TICK / 4.0f;
    odom->x += vx * cosf(odom->theta) - vy * sinf(odom->theta);
    odom->y += vx * sinf(odom->theta) + vy * cosf(odom->theta);
}
```

---

### Autonomous Navigation

R2 uses a **state machine** to execute a pre-defined mission sequence.

```c
// navigation.c — R2 autonomous state machine
typedef enum {
    STATE_IDLE = 0,
    STATE_MOVE_TO_HARVEST,
    STATE_HARVEST_RICE,
    STATE_MOVE_TO_SILO,
    STATE_DEPOSIT_RICE,
    STATE_RETURN_HOME,
    STATE_COMPLETE
} NavState_t;

void Navigation_Run(void) {
    switch (nav_state) {
        case STATE_IDLE:
            if (start_signal) nav_state = STATE_MOVE_TO_HARVEST;
            break;

        case STATE_MOVE_TO_HARVEST:
            if (MoveToPosition(HARVEST_X, HARVEST_Y, HARVEST_HEADING))
                nav_state = STATE_HARVEST_RICE;
            break;

        case STATE_HARVEST_RICE:
            Collector_Activate();
            if (HarvestComplete()) nav_state = STATE_MOVE_TO_SILO;
            break;

        case STATE_MOVE_TO_SILO:
            if (MoveToPosition(SILO_X, SILO_Y, SILO_HEADING))
                nav_state = STATE_DEPOSIT_RICE;
            break;

        case STATE_DEPOSIT_RICE:
            Depositor_Activate();
            if (DepositComplete()) nav_state = STATE_RETURN_HOME;
            break;

        case STATE_RETURN_HOME:
            if (MoveToPosition(HOME_X, HOME_Y, HOME_HEADING))
                nav_state = STATE_COMPLETE;
            break;

        default: break;
    }
}
```

---

### Remote Control (RC)

R1 reads RC input from a **FlySky FS-i6X** transmitter via the **iBUS protocol** over UART.

```c
// rc_receiver.c — Parse iBUS channel values
void RC_ParseIBUS(uint8_t *buf) {
    // iBUS frame: 0x20 0x40 | CH1_L CH1_H | CH2_L CH2_H | ...
    for (int ch = 0; ch < 10; ch++) {
        rc_channels[ch] = buf[2 + ch*2] | (buf[3 + ch*2] << 8);
    }
}

// Map channel value (1000–2000) to speed (-MAX_SPEED to +MAX_SPEED)
int16_t RC_MapToSpeed(uint16_t channel_val) {
    return (int16_t)((channel_val - 1500) * MAX_SPEED / 500);
}
```

---

## 🚀 Getting Started

### Prerequisites

- **STM32CubeIDE** (v1.12+)
- **ST-Link V2** programmer
- LiPo battery (3S or 4S, 2200–5000 mAh)
- CAN transceiver (TJA1050 or SN65HVD230)

### 1. Clone the Repository

```bash
git clone https://github.com/boyloy21/ROBOCON-ITC01-2024.git
cd ROBOCON-ITC01-2024
```

### 2. Open Project in STM32CubeIDE

1. Launch **STM32CubeIDE**
2. `File → Open Projects from File System`
3. Select either `R1_Robot/` or `R2_Robot/`
4. Click **Finish**

### 3. Verify Configuration

- Open the `.ioc` file and confirm pin assignments match your hardware
- Check the **CAN bit timing** matches your APB1 clock (default: 1 Mbps @ 42 MHz)
- Confirm **Timer settings** for PWM output match your motor drivers

### 4. Build and Flash

```
Build:  Ctrl + B  (or click the hammer icon)
Flash:  Run → Debug (F11) or Run → Run (Ctrl+F11)
```

### 5. First Power-On Checklist

- [ ] Verify all motor drivers are connected and powered
- [ ] Confirm CAN bus termination resistors (120Ω) at both ends
- [ ] Test RC controller link with R1 before enabling motor outputs
- [ ] Run R2 navigation in open space before field testing
- [ ] Verify Emergency STOP button is accessible and functional

---

## 📌 Pin Configuration

### R1 — STM32F407 Pin Map (Example)

| Pin | Function | Description |
|-----|----------|-------------|
| PA8 | TIM1_CH1 (PWM) | Motor 1 speed |
| PA9 | TIM1_CH2 (PWM) | Motor 2 speed |
| PA10 | TIM1_CH3 (PWM) | Motor 3 speed |
| PB0 | GPIO OUT | Motor 1 IN1 |
| PB1 | GPIO OUT | Motor 1 IN2 |
| PB2 | GPIO OUT | Motor 2 IN1 |
| PB3 | GPIO OUT | Motor 2 IN2 |
| PA11 | CAN1_RX | CAN Bus RX |
| PA12 | CAN1_TX | CAN Bus TX |
| PA2 | USART2_TX | RC UART TX (debug) |
| PA3 | USART2_RX | RC UART RX (iBUS input) |
| TIM3 | Encoder Mode | Motor 1 encoder |
| TIM4 | Encoder Mode | Motor 2 encoder |

### R2 — STM32F407 Pin Map (Example)

| Pin | Function | Description |
|-----|----------|-------------|
| PA8–PA11 | TIM1_CH1–CH4 (PWM) | Motor FL/FR/RL/RR speed |
| PB0–PB7 | GPIO OUT | Motor IN1/IN2 × 4 |
| PA11 | CAN1_RX | CAN Bus RX |
| PA12 | CAN1_TX | CAN Bus TX |
| PB6 | I2C1_SCL | IMU (MPU6050) |
| PB7 | I2C1_SDA | IMU (MPU6050) |
| TIM3–TIM5 | Encoder Mode | Wheel encoders |

> 📝 Full pin maps are defined in the `.ioc` files in each robot's project folder.

---

## 👥 Team Members

**Team ITC01 — Institute of Technology of Cambodia**

| Role | Responsibility |
|------|---------------|
| 🧠 Team Leader | Overall coordination, strategy |
| ⚙️ Embedded Software | STM32 firmware, motor control, PID |
| 🔌 Electronics | PCB design, wiring, power system |
| 🔩 Mechanical | Robot frame, mechanisms, CAD design |
| 🕹 Control System | Autonomous navigation, RC tuning |

> We are students from the **Department of Electrical & Energy Engineering**, Institute of Technology of Cambodia (ITC), Phnom Penh 🇰🇭

---

## 🤝 Contributing

This is an active competition project. Contributions and suggestions are welcome from future ITC teams.

1. Fork the repository
2. Create a branch: `git checkout -b feature/improvement-name`
3. Commit: `git commit -m "Improve: description"`
4. Push: `git push origin feature/improvement-name`
5. Open a **Pull Request**

### Future Improvement Ideas
- LiDAR-based obstacle avoidance for R2
- ROS2 integration for simulation testing
- Kalman Filter for IMU + encoder sensor fusion
- Wireless real-time telemetry dashboard

---

## 📚 References

- [ABU Robocon 2024 Official Rules — "Harvest Day"](https://www.abu.org.my/abu-robocon-2024/)
- [STM32F407 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32 HAL Driver Documentation](https://www.st.com/en/embedded-software/stm32cubef4.html)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [CAN Bus Specification — Bosch](https://www.kvaser.com/can-protocol-tutorial/)
- [Mecanum Wheel Kinematics Reference](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

---

## 📄 License

This project is licensed under the **MIT License** — open for learning, research, and future ITC Robocon teams.

---

<p align="center">
  <strong>🇰🇭 Representing Cambodia — Institute of Technology of Cambodia (ITC)</strong><br>
  <em>ABU Robocon 2024 "Harvest Day" — Quang Ninh, Vietnam</em><br><br>
  ⭐ Star this repo if it helped your team or inspired your build!
</p>
