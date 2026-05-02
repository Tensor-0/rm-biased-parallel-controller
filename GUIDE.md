# Wheel-Legged Robot — Project Orientation

> **Target audience**: Contributors with embedded C background, unfamiliar with the codebase.
> **Pre-requisites**: C (pointers, structs, `static`), FreeRTOS basics (task, `osDelayUntil`), control fundamentals (PID, state-space).

---

## 1. Physical System

A two-wheeled bipedal robot with biased-parallel leg linkages. Each leg has two joints (calf + thigh) driven by DM8009 motors, plus one M3508 hub motor at the foot. A BMI088 IMU provides 6-DOF inertial measurement. The control objective is dynamic balance — the system is inherently unstable (inverted pendulum) and must apply closed-loop correction at 1kHz.

---

## 2. Architecture Layers

| Layer | Role | Example |
|-------|------|---------|
| **Task** | Thread scheduling; control pipeline orchestration | `Control_Task`, `INS_Task`, `CAN_Task` |
| **Algorithm** | Pure math: PID, LQR, QuaternionEKF, filters | `lqr_controller.c`, `PID.c` |
| **Device** | Protocol drivers for specific peripherals | `Motor.c`, `Bmi088.c`, `Remote_Control.c` |
| **BSP** | STM32 HAL wrappers for on-chip peripherals | `bsp_can.c`, `bsp_spi.c` |

**I/O boundary pattern**: `control_io.h/c` is the sole interface between the Task layer and hardware globals. `Control_InputSnapshot_Update()` copies all sensors into `control_input_snapshot_t` at the top of each control cycle. `Control_OutputPacket_Generate()` writes computed motor commands into `g_motor_cmd` at the cycle end. No other module accesses raw hardware variables directly.

---

## 3. Control Pipeline (16 steps, 1ms cycle)

```
1. Snapshot acquisition
2. Low-voltage check
3. Mode state machine (WEAK → joint reduction → BALANCE)
4. Joint angle coordinate mapping
5. VMC forward kinematics (θ₁,θ₂ → L₀,φ₀)
6. LQR gain update (K = f(L₀) via Horner)
7. Sensor fusion (IMU + wheel → 6-DOF state)
8. Chassis velocity control
9. Chassis height control
10. Roll compensation
11. Leg-length PID + gravity compensation
12. Lift-off detection (FN < 22 N threshold)
13. LQR control output (u = −K·x)
14. Jacobian torque mapping (F,Tp → 4 joint torques)
15. Command pack to g_motor_cmd
16. VOFA+ telemetry transmit
```

---

## 4. Key Data Structures

- **`Control_Info_Typedef`** (`Control_Task.h`): Global control state container. Contains left/right `Leg_Info_Typedef` instances, chassis FSM state, init flags, yaw error, roll parameters.
- **`Leg_Info_Typedef`** (`Control_Task.h`): Per-leg full state: BiasedParams, Target/Measure 6-DOF vectors, Support flags, Moment components, Velocity pipeline, SendValue.
- **`motor_command_packet_t`** (`control_io.h`): Output boundary struct. 4 × joint torque (float), 2 × wheel current (int16), chassis/init/motor_active flags, seq number, tick.
- **`control_input_snapshot_t`** (`control_io.h`): Input boundary struct. INS_Info (full copy), remote channels, 4 joint snapshots, 2 wheel snapshots, battery voltage, tick.

---

## 5. Recommended Reading Order

| Order | File | Focus |
|-------|------|-------|
| 1 | `Application/Task/Inc/Control_Task.h` | Understand data structures |
| 2 | `Application/Task/Src/Control_Task.c` | Read the orchestrator (105 lines) |
| 3 | `Application/Task/Src/mode_state_machine.c` | Init sequence + PID params |
| 4 | `Application/Task/Src/vmc_kinematics.c` | Forward kinematics derivation |
| 5 | `Application/Task/Src/lqr_controller.c` | Gain-scheduled LQR |
| 6 | `Application/Task/Src/sensor_fusion.c` | Velocity estimation |
| 7 | `Application/Task/Src/chassis_control.c` | Chassis command channels |
| 8 | `Application/Task/Src/CAN_Task.c` | Motor protocol dispatch |
| 9 | `Application/Task/Src/INS_Task.c` | IMU + QuaternionEKF |
| 10 | `Application/Task/Src/control_io.c` | I/O isolation implementation |

---

## 6. Parameter Modification Guide

| Parameter | Location (v3.1) | Effect |
|-----------|----------------|--------|
| Link lengths, masses, wheel radius | `Robot_Config.h` | Physical model; affects VMC output |
| PID Kp/Ki/Kd arrays | `mode_state_machine.c` | Response stiffness/damping |
| LQR K-coefficients (K11–K26) | `lqr_controller.c` | Balance authority; regenerate via `get_K.m` or `tools/lqr_gain_schedule.py` |
| Q/R diagonal weights | `robot_params.m` / `tools/lqr_gain_schedule.py` | Trade-off: state regulation vs. control effort |
| Fusion α (0.8) | `sensor_fusion.c` | Velocity filter bandwidth |
| Lift-off threshold (22 N) | `vmc_kinematics.c` (VMC_Measure_F_Tp_Calculate) | Contact detection sensitivity |
| Joint torque clamp (15 N·m) | `vmc_kinematics.c` (VMC_Joint_Tourgue_Calculate) | Overcurrent protection |

---

## 7. Onboarding Steps (Ubuntu 22.04)

```bash
# Clone and switch to experimental branch
git clone https://github.com/Tensor-0/rm-biased-parallel-controller.git
cd rm-biased-parallel-controller
git checkout v3.1-improvements

# Build simulation core (no hardware)
cd simulation_core && mkdir -p build && cd build
cmake .. && make -j4

# Test algorithm
python3 -c "
from wheel_legged_sim import WheelLeggedDynamics, RobotState
wld = WheelLeggedDynamics()
state = RobotState()
state.state_vector = [0, 0, 0, 0, 0.05, 0]
state.joint_angles = [0, -0.13, 0.13, 0]
cmd = wld.compute_torques(state)
print(cmd.joint_torque)
"
```

---

## 8. Branch Strategy

```
main                     ← stable (vehicle-verified)
v3.1-improvements        ← experimental (params consolidated, MATLAB refactored, Python alternative)
feature/sim-core-cpp     ← simulation (C++17 library, pybind11, Mujoco-ready)
```

Feature branches: `feature/<name>`, `bugfix/<name>`, `chore/<name>`, `experiment/<name>`. No direct commits to `main`. All merges via Pull Request.

---

## 9. Common Issues

| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| Compile error: undefined ref to new `.c` file | File not added to Keil/EIDE project | Manually add to project |
| Robot oscillates on power-up | IMU orientation reversed | Check sign of `Angle[2]` in `sensor_fusion.c` line 17 |
| Remote unresponsive | SBUS frame incomplete | Verify USART DMA double-buffer config |
| `arm_sqrt_f32` divide-by-zero | `S_Radicand` negative (floating-point rounding) | Enforced by `if(S < 1e-6)` guard (PR #18) |
| Joint motors stall during init | Joint reduction timeout | v3.1 adds 5-second timeout with forced WEAK transition |
