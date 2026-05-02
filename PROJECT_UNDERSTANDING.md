# Project Architecture — Global Comprehension Summary

> **Audience**: Engineers seeking to understand the full system before making modifications.
> **Scope**: Physical abstraction, software layering, mathematical foundation, parameter tuning, quality assessment.

---

## 1. Physical Problem

A biased-parallel dual-wheel-legged robot with two joints per leg (calf + thigh, driven by DM8009 motors) and one hub motor per foot (M3508). The system is inherently unstable — an inverted pendulum with a moving pivot. The control objective is to maintain dynamic balance while responding to chassis velocity and yaw-rate commands from an SBUS remote.

### 1.1 Three-Layer Physical Abstraction

```
Real Mechanism        →    VMC Reduction        →    LQR Control
══════════════             ════════════             ══════════
4 joint angles             1 virtual length L₀      6-DOF state vector
4 joint torques            1 virtual inclination φ₀  2 control inputs
closed-form kinematics     exact mapping             u = −K·x
```

The VMC forward kinematics is an exact geometric transformation — no approximation. The LQR control law is computed offline, with gain scheduling at runtime.

---

## 2. Software Architecture

### 2.1 Four-Layer Separation

| Layer | Responsibility | Key Files |
|-------|---------------|-----------|
| Task | Thread scheduling; control pipeline orchestration | `Control_Task.c`, `INS_Task.c`, `CAN_Task.c` |
| Algorithm | Pure math: PID, LQR, QuaternionEKF, filters | `lqr_controller.c`, `PID.c`, `Quaternion.c` |
| Device | Protocol drivers for specific peripherals | `Motor.c`, `Bmi088.c`, `Remote_Control.c` |
| BSP | STM32 HAL wrappers for on-chip peripherals | `bsp_can.c`, `bsp_spi.c` |

### 2.2 I/O Boundary Pattern

`control_io.h/c` is the sole interface between Task layer and hardware globals:

- **Input snapshot** (`control_input_snapshot_t`): Copied once per cycle from `INS_Info`, `remote_ctrl`, `DM_8009_Motor[]`, `Chassis_Motor[]`. The 16-step pipeline reads only this snapshot.
- **Output packet** (`motor_command_packet_t`): Populated at cycle end from `Control_Info.SendValue`. `CAN_Task` reads only this packet — it has no dependency on `Control_Info`, `INS_Info`, or `remote_ctrl` (v3.0 decoupling).

This eliminates the data race between interrupt-driven sensor updates and RTOS-thread control logic.

### 2.3 Control Pipeline (16 steps, 1ms)

```
Snapshot → Low-V → ModeFSM → JointMap → VMC-FK → LQR-K → Fusion
→ Move → Height → Roll → LegLen → LiftOff → LQR-X → LQR-u
→ Jacobian → Pack → Telemetry
```

### 2.4 Thread Collaboration

| Thread | Frequency | Input | Output |
|--------|----------|-------|--------|
| INS_Task | 1kHz | BMI088 SPI | `INS_Info` (global) |
| Control_Task | 1kHz | Snapshot (copy of `INS_Info`) | `g_motor_cmd` (global) |
| CAN_Task | 1kHz | `g_motor_cmd` + motor feedback | FDCAN2/3 |

Producer-consumer pattern: each thread has a single responsibility and a single output channel.

---

## 3. Mathematical Foundation

### 3.1 LQR: Algebraic Riccati Equation

```
Given: ẋ = Ax + Bu
Find:  u = −Kx minimizing ∫(xᵀQx + uᵀRu)dt

K = R⁻¹BᵀP  where  AᵀP + PA − PBR⁻¹BᵀP + Q = 0
```

Q diagonal weights encode state regulation priority:
- Q(5,5) = 5000: body tilt φ is the dominant instability
- Q(3,3) = 500: position drift is a secondary concern
- R(2,2) / R(1,1) = 0.25: hip torque is 4× cheaper than wheel torque

### 3.2 Gain Scheduling

K is not constant — longer legs produce larger moment arms, requiring smaller gains:

```
K(L₀) = c₁·L₀³ + c₂·L₀² + c₃·L₀ + c₄
```

12 coefficient sets (K11–K26) are pre-computed offline via MATLAB `lqr()` over 31 leg-length samples (0.10–0.40 m, step 0.01 m), then fitted to cubic polynomials via `polyfit()`. At runtime, Horner's method evaluates the polynomial in 3 multiplications and 3 additions per coefficient.

### 3.3 VMC Forward Kinematics

Exact closed-form mapping from 2 joint angles (θ₁ calf, θ₂ thigh) to virtual leg parameters (L₀, φ₀) using linkage geometry:

```
M = (θ₁ − θ₂) / 2
N = (θ₁ + θ₂) / 2
S = √(b² − a²·sin²(M))
L₀ = (a·cos(M) + S) / K
φ₀ = N
```

No approximation — the mapping is exact for the given linkage model.

---

## 4. Parameter Configuration

| Parameter | Location (v3.1) | Sensitivity |
|-----------|----------------|-------------|
| Body mass M | `Robot_Config.h` CONF_BODY_MASS + `robot_params.m` M1 | 🔴 High — requires LQR recomputation |
| Gravity compensation | `Robot_Config.h` CONF_GRAVITY_COMPENSATION | 🟡 Medium |
| Leg length range | `Robot_Config.h` CONF_LEG_LENGTH_MIN/MAX | 🟡 Medium |
| Q/R diagonal weights | `robot_params.m` Q_val/R_val | 🔴 High — requires LQR recomputation |
| PID parameters | `mode_state_machine.c` PID_PARAM macros | 🟡 Medium |
| Fusion α (0.8) | `sensor_fusion.c` CONF_FUSION_ALPHA | 🟢 Low |

---

## 5. Quality Assessment (v3.0)

| Dimension | Rating | Notes |
|-----------|--------|-------|
| Algorithm depth | ★★★★★ | VMC+LQR dual-layer; academic-grade derivation |
| Modularity (post-#17) | ★★★★☆ | 5-domain module split; `control_io.h` I/O boundary |
| Real-time safety (post-#18) | ★★★★☆ | P0 data race eliminated; sqrt zero-guard; snapshot size assertion |
| Code readability (post-#21) | ★★★★★ | 10 core files with engineering-line comments |
| Documentation | ★★★★★ | README + GUIDE + architecture report + MATLAB guide + this document |
| Offline toolchain (v3.1) | ★★★★★ | MATLAB + GNU Octave + Python — three equivalent paths |
| Parameter management (v3.1) | ★★★★★ | `Robot_Config.h` + `robot_params.m` — single-source configuration |

---

## 6. Next Steps

| Priority | Task | Branch |
|----------|------|--------|
| 🔴 P0 | Vehicle validation of v3.0 changes | `main` |
| 🟡 P1 | Vehicle validation of v3.1 improvements | `v3.1-improvements` |
| 🟡 P1 | Full bring-up sequence test | `main` |
| 🟢 P2 | FreeRTOS Queue replacement for shared variables | `v3.1-improvements` (`#define USE_QUEUE`) |
| 🟢 P3 | Jump/step-over capability | new feature branch |
