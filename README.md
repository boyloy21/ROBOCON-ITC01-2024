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
- [Software Modules](#-software-modules)
  - [Motor Control](#motor-control)
  - [PID Controller](#pid-controller)
  - [MPC Controller (V1)](#mpc-controller-v1)
  - [CAN Bus Communication](#can-bus-communication)
  - [IMU & Odometry](#imu--odometry)
  - [Autonomous Navigation & Perception](#autonomous-navigation--perception)
  - [Remote Control (RC)](#remote-control-rc)
- [Result Videos](#-result-videos)
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
│  MR1 operates in Area 1 & 2      MR2 operates in Area 2 & 3 │
└──────────────────────────────────────────────────────────────┘
```

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

##### MR1 V1 Drive Kinematics (4-Wheel Mecanum)

```
      FL Motor ╔═══════╗ FR Motor
        (↗)    ║       ║   (↘)
               ║  MR1  ║
        (↘)    ║       ║   (↗)
      RL Motor ╚═══════╝ RR Motor

  Vx  =  (V_FL + V_FR + V_RL + V_RR) / 4
  Vy  =  (-V_FL + V_FR + V_RL - V_RR) / 4
  Wz  =  (-V_FL + V_FR - V_RL + V_RR) / (4 * (Lx + Ly))
```

---

#### MR1 — V2: 4-Wheel Omni Drive (Alternate)

| Feature | Detail |
|---------|--------|
| Drive System | **4-Wheel Omni Drive** |
| Control Mode | Semi-autonomous (manual RC + auto assist) |
| Operating Area | Area 1 (Planting) + Area 2 (Harvesting) |
| Main MCU | STM32F4xx |
| Motor Type | DC Brushed Motor with Encoder |
| Communication | CAN Bus (inter-board) + RC Receiver |

##### MR1 V2 Drive Kinematics (4-Wheel Omni)

```
        Motor 1 (90°)
             ↑
  Motor 4  ╔═══════╗  Motor 2
   (180°)  ║       ║   (0°)
           ║  MR1  ║
           ╚═══════╝
        Motor 3 (270°)

  Vx  =  (-V1 + V3) / 2
  Vy  =  (-V2 + V4) / 2
  Wz  =  (V1 + V2 + V3 + V4) / (4 * R)
```

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

#### MR2 Drive Kinematics (4-Wheel Omni)

```
        Motor 1 (90°)
             ↑
  Motor 4  ╔═══════╗  Motor 2
   (180°)  ║       ║   (0°)
           ║  MR2  ║
           ╚═══════╝
        Motor 3 (270°)

  Vx  =  (-V1 + V3) / 2
  Vy  =  (-V2 + V4) / 2
  Wz  =  (V1 + V2 + V3 + V4) / (4 * R)
```

---

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

##### Perception Logic (Pseudocode)

```c
// perception.c — Ball detection and color classification
BallResult_t Perception_DetectBall(RealSenseFrame_t *frame) {
    BallResult_t result = {0};

    // Step 1: Detect ball in RGB frame
    if (!ColorDetector_FindBall(frame->rgb, &result.bbox)) {
        return result;   // No ball found
    }

    // Step 2: Classify color
    ColorClass_t color = ColorDetector_Classify(frame->rgb, &result.bbox);

    if (color == COLOR_RED || color == COLOR_BLUE) {
        result.valid     = true;          // ✅ Proceed to catch
        result.color     = color;
        result.distance  = DepthEstimate(frame->depth, &result.bbox);
    } else {
        // COLOR_PURPLE or unknown → ignore
        result.valid     = false;         // ❌ Skip
    }

    return result;
}
```

##### MR2 Autonomous State Machine

```c
// navigation.c — MR2 autonomous state machine
typedef enum {
    STATE_IDLE = 0,
    STATE_SCAN_FOR_BALL,        // Rotate/search for a valid ball
    STATE_MOVE_TO_BALL,         // Navigate toward detected ball
    STATE_CATCH_BALL,           // Activate collector mechanism
    STATE_MOVE_TO_SILO,         // Navigate to correct silo (by color)
    STATE_DEPOSIT_BALL,         // Release ball into silo
    STATE_RETURN_TO_SCAN,       // Return to scanning position
    STATE_COMPLETE
} NavState_t;

void Navigation_Run(void) {
    switch (nav_state) {
        case STATE_IDLE:
            if (start_signal) nav_state = STATE_SCAN_FOR_BALL;
            break;

        case STATE_SCAN_FOR_BALL: {
            BallResult_t ball = Perception_DetectBall(&current_frame);
            if (ball.valid) {
                target_ball  = ball;
                nav_state    = STATE_MOVE_TO_BALL;
            }
            // else: keep scanning (rotate slowly)
            break;
        }

        case STATE_MOVE_TO_BALL:
            if (MoveTowardBall(&target_ball))
                nav_state = STATE_CATCH_BALL;
            break;

        case STATE_CATCH_BALL:
            Collector_Activate();
            if (CatchComplete()) nav_state = STATE_MOVE_TO_SILO;
            break;

        case STATE_MOVE_TO_SILO: {
            Position_t silo_pos = GetSiloPosition(target_ball.color);
            if (MoveToPosition(silo_pos.x, silo_pos.y, silo_pos.heading))
                nav_state = STATE_DEPOSIT_BALL;
            break;
        }

        case STATE_DEPOSIT_BALL:
            Depositor_Activate();
            if (DepositComplete()) nav_state = STATE_RETURN_TO_SCAN;
            break;

        case STATE_RETURN_TO_SCAN:
            if (MoveToPosition(SCAN_X, SCAN_Y, SCAN_HEADING))
                nav_state = STATE_SCAN_FOR_BALL;
            break;

        default: break;
    }
}
```

---

#### MR2 Mechanical Mechanisms
- **Ball Collector** — scoops and collects balls (Red / Blue) detected by camera
- **4-Wheel Omni Drive Base** — omnidirectional autonomous navigation
- **Silo Depositor** — transfers collected balls into the correct color-matched silo
- **Intel RealSense Mount** — fixed front-facing camera bracket for perception

---

## 📁 Repository Structure

```
ROBOCON-ITC01-2024/
│
├── MR1_Robot/                      # Manual Robot — Semi-autonomous
│   ├── V1_Mecanum/                 # Primary: 4-Wheel Mecanum Drive
│   │   ├── Core/
│   │   │   ├── Src/
│   │   │   │   ├── main.c
│   │   │   │   ├── motor_control.c
│   │   │   │   ├── pid_controller.c
│   │   │   │   ├── mecanum_drive.c     # 4-wheel mecanum kinematics
│   │   │   │   ├── can_comm.c
│   │   │   │   └── rc_receiver.c
│   │   │   └── Inc/
│   │   └── MR1_V1.ioc
│   │
│   └── V2_Omni/                    # Alternate: 4-Wheel Omni Drive
│       ├── Core/
│       │   ├── Src/
│       │   │   ├── main.c
│       │   │   ├── motor_control.c
│       │   │   ├── pid_controller.c
│       │   │   ├── omni4_drive.c       # 4-wheel omni kinematics
│       │   │   ├── can_comm.c
│       │   │   └── rc_receiver.c
│       │   └── Inc/
│       └── MR1_V2.ioc
│
├── MR2_Robot/                      # Autonomous Robot — Fully Autonomous
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c              # Main loop & state machine
│   │   │   ├── motor_control.c
│   │   │   ├── omni4_drive.c       # 4-wheel omni kinematics
│   │   │   ├── imu_driver.c        # IMU / gyroscope interface
│   │   │   ├── odometry.c          # Encoder-based position tracking
│   │   │   ├── navigation.c        # Autonomous state machine
│   │   │   ├── perception.c        # RealSense color & ball detection
│   │   │   ├── pid_controller.c    # PID speed/position loop (V2 — final)
│   │   │   ├── mpc_controller.c    # MPC trajectory control (V1 — testing)
│   │   │   └── can_comm.c
│   │   └── Inc/
│   └── MR2_Robot.ioc
│
├── Shared/                         # Code shared between MR1 and MR2
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
│                       MR1 (Manual Robot)                   │
│                                                            │
│  RC Receiver ──→ STM32 Main MCU ──→ CAN Bus               │
│                      │                                     │
│              ┌───────┼────────┐                            │
│              ↓       ↓        ↓       ↓                    │
│           Motor FL  Motor FR  Motor RL  Motor RR           │
│          (Encoder) (Encoder) (Encoder) (Encoder)           │
│           ← H-Bridge Drivers / PID Speed Loop →            │
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
| MCU | STM32F407VGT6 / STM32F401 |
| Clock | Up to 168 MHz (STM32F407) |
| Flash | 1 MB |
| RAM | 192 KB |
| IDE | STM32CubeIDE + STM32CubeMX |
| Debugger | ST-Link V2 |

### Actuators & Drivers

| Component | Role | Qty |
|-----------|------|-----|
| DC Gear Motor + Encoder | Drive wheels (MR1: 4, MR2: 4) | 8 |
| L298N / BTS7960 H-Bridge | Motor driver | 4–8 |
| Servo Motor (MG996R) | Gripper & mechanisms | 2–4 |
| BLDC Motor + ESC | High-speed mechanism (if applicable) | TBD |

### Sensors & Peripherals

| Component | Purpose |
|-----------|---------|
| **Intel RealSense Camera** | Ball detection & color classification for MR2 |
| MPU6050 / ICM-20602 IMU | Heading / gyroscope for MR2 navigation |
| Incremental Encoder (600 PPR) | Motor speed & position feedback |
| TJA1050 CAN Transceiver | CAN Bus inter-board communication |
| FlySky FS-i6X RC Transmitter | Manual control of MR1 |
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

Discrete PID implementation for closed-loop speed regulation — used in **MR2 V2 (Final)** and **MR1**.

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

| Robot | Version | Kp | Ki | Kd |
|-------|---------|----|----|----|
| MR1 (Mecanum V1) | — | 2.0 | 0.5 | 0.05 |
| MR1 (Omni V2)    | — | 2.0 | 0.5 | 0.05 |
| MR2 (Omni — PID V2) | Final | 2.5 | 0.8 | 0.08 |

---

### MPC Controller (V1)

Model Predictive Control tested for **MR2 V1** — predicts future states over a horizon and minimises a cost function for smoother trajectory tracking.

```c
// mpc_controller.c — MPC compute (simplified horizon loop)
void MPC_Compute(MPC_t *mpc, float x_ref[], float x_curr[]) {
    // Predict states over horizon N using discrete model
    for (int k = 0; k < MPC_HORIZON; k++) {
        mpc->X_pred[k+1] = A * mpc->X_pred[k] + B * mpc->U[k];
    }
    // Minimise cost: J = sum(||x_ref - x_pred||_Q + ||u||_R)
    MPC_SolveQP(mpc);   // Quadratic programming solver
    // Apply first control input
    mpc->output = mpc->U[0];
}
```

> ⚠️ **Note:** MPC V1 is used for research/testing purposes. The **PID V2** build is the final competition firmware.

---

### CAN Bus Communication

All boards communicate via **CAN 1 Mbps**. Each robot has a defined set of message IDs.

```c
// can_protocol.h — Message ID definitions
#define CAN_ID_MR1_SPEED_CMD    0x101   // MR1 → Motor Board: speed setpoints
#define CAN_ID_MR1_STATUS       0x102   // MR1 Motor Board → Main: encoder feedback
#define CAN_ID_MR2_SPEED_CMD    0x201   // MR2 → Motor Board: speed setpoints
#define CAN_ID_MR2_STATUS       0x202   // MR2 Motor Board → Main: encoder feedback
#define CAN_ID_MR2_NAV_STATE    0x210   // MR2 Navigation state broadcast
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

Used by **MR2** for autonomous heading control and dead-reckoning position tracking.

```c
// imu_driver.c — Read yaw from MPU6050 via I2C
float IMU_GetYaw(void) {
    int16_t raw_gz;
    MPU6050_ReadGyroZ(&raw_gz);
    float gz_dps = raw_gz / 131.0f;               // Convert to °/s
    yaw += gz_dps * SAMPLE_TIME_S;                 // Integrate
    return yaw;
}

// odometry.c — Update robot position from 4-wheel omni encoder ticks
void Odometry_Update(Odometry_t *odom, int32_t enc_1, int32_t enc_2,
                                        int32_t enc_3, int32_t enc_4) {
    float vx =  (-enc_1 + enc_3) * DIST_PER_TICK / 2.0f;
    float vy =  (-enc_2 + enc_4) * DIST_PER_TICK / 2.0f;
    odom->x += vx * cosf(odom->theta) - vy * sinf(odom->theta);
    odom->y += vx * sinf(odom->theta) + vy * cosf(odom->theta);
}
```

---

### Autonomous Navigation & Perception

See the full pipeline and state machine in the [MR2 section above](#mr2-autonomous-robot-fully-autonomous).

Key components:
- **Intel RealSense** — provides RGB + depth frames
- **Color Classifier** — identifies Red / Blue (valid) vs Purple (ignore)
- **Navigation State Machine** — transitions from scan → move → catch → silo → deposit
- **MPC (V1)** / **PID (V2 Final)** — trajectory and speed control

---

### Remote Control (RC)

MR1 reads RC input from a **FlySky FS-i6X** transmitter via the **iBUS protocol** over UART.

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

## 🎥 Result Videos

Testing videos for **MR1** and **MR2** are documented in the GitHub issue below:

> 📹 **[View MR1 & MR2 Testing Result Videos → Issue #2](https://github.com/boyloy21/ROBOCON-ITC01-2024/issues/2)**

The issue contains:
- 🤖 **MR1** — Mecanum V1 and Omni V2 drive testing footage
- 🤖 **MR2** — Fully autonomous pipeline testing (MPC V1 vs PID V2)
- 🎯 Ball detection & color classification demos (RealSense camera)
- 🏆 Field runs and competition preparation clips

---

## 🚀 Getting Started

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

## 📌 Pin Configuration

### MR1 — STM32F407 Pin Map (4-Wheel Drive)

| Pin | Function | Description |
|-----|----------|-------------|
| PA8 | TIM1_CH1 (PWM) | Motor FL speed |
| PA9 | TIM1_CH2 (PWM) | Motor FR speed |
| PA10 | TIM1_CH3 (PWM) | Motor RL speed |
| PA11 | TIM1_CH4 (PWM) | Motor RR speed |
| PB0 | GPIO OUT | Motor FL IN1 |
| PB1 | GPIO OUT | Motor FL IN2 |
| PB2 | GPIO OUT | Motor FR IN1 |
| PB3 | GPIO OUT | Motor FR IN2 |
| PA11 | CAN1_RX | CAN Bus RX |
| PA12 | CAN1_TX | CAN Bus TX |
| PA2 | USART2_TX | RC UART TX (debug) |
| PA3 | USART2_RX | RC UART RX (iBUS input) |
| TIM3 | Encoder Mode | Motor FL encoder |
| TIM4 | Encoder Mode | Motor FR encoder |

### MR2 — STM32F407 Pin Map (4-Wheel Omni + RealSense)

| Pin | Function | Description |
|-----|----------|-------------|
| PA8–PA11 | TIM1_CH1–CH4 (PWM) | Motor 1/2/3/4 speed |
| PB0–PB7 | GPIO OUT | Motor IN1/IN2 × 4 |
| PA11 | CAN1_RX | CAN Bus RX |
| PA12 | CAN1_TX | CAN Bus TX |
| PB6 | I2C1_SCL | IMU (MPU6050) |
| PB7 | I2C1_SDA | IMU (MPU6050) |
| TIM3–TIM5 | Encoder Mode | Wheel encoders |
| USB OTG / UART | RealSense Interface | Camera data stream to perception module |

> 📝 Full pin maps are defined in the `.ioc` files in each robot's project folder.

---

## 👥 Team Members

**Team ITC01 — Institute of Technology of Cambodia**

| Role | Responsibility |
|------|---------------|
| 🧠 Team Leader | Overall coordination, strategy |
| ⚙️ Embedded Software | STM32 firmware, motor control, PID / MPC |
| 🔌 Electronics | PCB design, wiring, power system |
| 🔩 Mechanical | Robot frame, mechanisms, CAD design |
| 🕹 Control System | Autonomous navigation, perception, RC tuning |
| 👁 Computer Vision | RealSense integration, color detection |

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
- LiDAR-based obstacle avoidance for MR2
- ROS2 integration for simulation testing
- Kalman Filter for IMU + encoder sensor fusion
- Extended Kalman Filter fusing RealSense depth + odometry
- Wireless real-time telemetry dashboard
- Full MPC deployment replacing PID on competition build

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

This project is licensed under the **MIT License** — open for learning, research, and future ITC Robocon teams.

---

<p align="center">
  <strong>🇰🇭 Representing Cambodia — Institute of Technology of Cambodia (ITC)</strong><br>
  <em>ABU Robocon 2024 "Harvest Day" — Quang Ninh, Vietnam</em><br><br>
  ⭐ Star this repo if it helped your team or inspired your build!
</p>
