# rm-biased-parallel-controller

> **偏置并联轮腿机器人 — 实时平衡控制系统**
> Target: STM32H723VGTx (Cortex-M7, 550MHz) | RTOS: FreeRTOS (CMSIS-OS v2) | Control loop: 1kHz

---

> 🔒 **Branch: `main` — stable (vehicle-validated)**
> 🚧 **Branch: [`v3.1-improvements`](https://github.com/Tensor-0/rm-biased-parallel-controller/tree/v3.1-improvements) — experimental**
> 🧪 **Branch: [`feature/sim-core-cpp`](https://github.com/Tensor-0/rm-biased-parallel-controller/tree/feature/sim-core-cpp) — simulation**

---

### Core Algorithm Modules (v3.0 refactor)

| Module | Source | Role |
|--------|--------|------|
| I/O Boundary | `Application/Task/Src/control_io.c` | Snapshot-based input acquisition; `motor_command_packet_t` output packing. Isolates control logic from hardware globals. |
| Mode FSM | `Application/Task/Src/mode_state_machine.c` | PID initialization; low-voltage check; joint-reduction-to-balance state transitions. |
| VMC Kinematics | `Application/Task/Src/vmc_kinematics.c` | Closed-chain forward kinematics: joint angles → virtual leg length L₀ and inclination φ₀. Jacobian-based torque mapping. |
| LQR Controller | `Application/Task/Src/lqr_controller.c` | Gain-scheduled LQR: polynomial K(L₀) via Horner's method → u = −K·x. Support-flag-adaptive gain zeroing. |
| Sensor Fusion | `Application/Task/Src/sensor_fusion.c` | 1st-order lag filter (α = 0.8) fusing wheel odometry and IMU-derived chassis acceleration. 6-DOF state vector construction. |
| Chassis Control | `Application/Task/Src/chassis_control.c` | Velocity/height/roll/leg-length PID channels with ramp-based smoothing. |
| CAN Dispatch | `Application/Task/Src/CAN_Task.c` | Reads `g_motor_cmd` only. MIT-mode torque commands to DM8009 joints; current commands to M3508 wheels via FDCAN2/3. |
| INS | `Application/Task/Src/INS_Task.c` | BMI088 readout → 2nd-order LPF on accelerometer → Quaternion EKF → Euler angles. Yaw total-angle unwrapping. |

### 🐧 Developer Navigation (Ubuntu 22.04)

| Objective | Reference | Dependencies | Est. Time |
|-----------|-----------|-------------|-----------|
| Algorithm derivation & system model | `PROJECT_UNDERSTANDING.md` | None | 20 min |
| Local physics simulation | `SIMULATION_GUIDE.md` (branch `feature/sim-core-cpp`) | `pip install pybind11` | 30 min |
| LQR gain computation (MATLAB-free) | `MATLAB_GUIDE.md` §8 or `tools/lqr_gain_schedule.py` | `numpy`, `scipy` | 10 min |
| Project orientation | `GUIDE.md` | None | 15 min |
| Architecture-level Code Review | `MDK-ARM/architecture_analysis_report.md` | None | 10 min |
| Refactoring roadmap | `MDK-ARM/architecture_refactor_plan.md` | None | 10 min |

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────┐
│  Task Layer (1kHz, FreeRTOS threads)                 │
│  INS_Task ──▶ Control_Task (orchestrator) ──▶ CAN_Task│
├──────────────────────────────────────────────────────┤
│  Algorithm Layer: PID · LQR · QuaternionEKF · LPF    │
├──────────────────────────────────────────────────────┤
│  Device Layer: DM8009 · M3508 · BMI088 · SBUS        │
├──────────────────────────────────────────────────────┤
│  BSP Layer: FDCAN · SPI · UART · ADC · PWM           │
├──────────────────────────────────────────────────────┤
│  STM32H723VGTx (Cortex-M7, 550MHz)                   │
└──────────────────────────────────────────────────────┘
```

**Data flow**: Sensor interrupts update global structs → `Control_InputSnapshot_Update()` copies into `control_input_snapshot_t` → 16-step pipeline → `Control_OutputPacket_Generate()` writes `g_motor_cmd` → `CAN_Task` transmits via FDCAN. `CAN_Task` has no direct dependency on `Control_Info`, `INS_Info`, or `remote_ctrl` (v3.0 decoupling).

---

## 2. Control Algorithm

### 2.1 VMC Forward Kinematics

The biased-parallel leg mechanism (2 joints: calf θ₁, thigh θ₂) is reduced to an equivalent single-link inverted pendulum via closed-form geometry:

```
M = (θ₁ − θ₂) / 2          (half-difference angle)
N = (θ₁ + θ₂) / 2          (half-sum angle)
S = √(b² − a²·sin²(M))     (linkage projection)
t = a·cos(M) + S            (horizontal span)
A = a·t·sin(M) / S          (Jacobian coefficient)

L₀ = t / K                 (virtual leg length)
φ₀ = N                      (virtual leg inclination)
```

where `a = AD` (calf link length), `b = AH` (thigh link length), `K = AD/AH`. The mapping is exact—no approximation.

### 2.2 LQR Gain-Scheduled Balance Control

**State vector** (6-DOF): `x = [θ, dθ, x, dx, φ, dφ]`

**Gain scheduling**: K(L₀) is a cubic polynomial in virtual leg length, pre-computed offline via MATLAB `lqr()` with Q = diag([30, 1, 500, 100, 5000, 1]), R = diag([1, 0.25]). At runtime, Horner's method evaluates:

```
K[i][j](L₀) = c₁·L₀³ + c₂·L₀² + c₃·L₀ + c₄
```

**Control law**: `u = −K·x`, where u = [T (wheel torque), Tp (hip torque)].

**v3.0 optimization**: 72 `powf()` calls per cycle (1kHz) replaced with 6 pre-computed monomials (L₀², L₀³), reducing FPU load by ~36 multiplications per leg.

**v3.0 memory optimization**: Intermediate geometric variables (a, b, M, N, S, S_radicand, t) moved from `Leg_Info_Typedef` to stack-local scope in `VMC_Calculate()`, recovering ~112 bytes of `.bss`.

**Adaptive zeroing**: Legs with `Support.Flag == 1` (ground reaction force FN < 22 N) have K[0][*] and K[1][2:5] zeroed to suppress ineffective wheel torque and partial joint torque.

### 2.3 Jacobian Torque Mapping

Virtual forces (F, Tp) are mapped to physical joint torques via the 2×2 Jacobian inverse:

```
Left leg:   T_calf  = (A/K)·F + Tp/2
            T_thigh = (−A/K)·F + Tp/2
Right leg:  T_thigh = (A/K)·F + Tp/2
            T_calf  = (−A/K)·F + Tp/2
```

Outputs clamped to ±15 N·m. Wheel torque T is converted to M3508 current via coefficient ×1200, clamped to ±10000 mA.

### 2.4 Sensor Fusion

Chassis velocity is estimated via 1st-order lag filter:

```
Body  = wheel_RPM × (π/30) / reduction_ratio × wheel_radius
Predict = Fusion_prev + Accel × Δt   (Δt = 0.001s)
Fusion  = 0.8·Predict + 0.2·Body
```

Position is integrated from Fusion only when `Target_Velocity == 0`. Acceleration is derived from IMU accelerometer with gravity and centripetal terms subtracted.

---

## 3. Build & Deploy

### 3.1 Keil MDK-ARM (Windows)

- Keil MDK v5.38+, STM32H7xx device pack
- Open `MDK-ARM/COD_H7_Template.uvprojx`
- Build (F7), Download (F8)

### 3.2 VS Code + GCC (Linux, experimental)

- arm-none-eabi-gcc 10.3.1+, EIDE plugin
- Open project root; EIDE auto-detects `.eide` configuration

### 3.3 Simulation (no hardware required)

```bash
git checkout feature/sim-core-cpp
cd simulation_core && mkdir -p build && cd build
cmake .. && make -j4
python3 -c "from wheel_legged_sim import WheelLeggedDynamics,RobotState; ..."
```

See `SIMULATION_GUIDE.md` on branch `feature/sim-core-cpp` for the full walkthrough.

---

## 4. Directory Structure

```
├── Application/Task/Inc/          # Module headers + Robot_Config.h (v3.1)
├── Application/Task/Src/          # 5 control modules + I/O boundary + CAN/INS tasks
├── Components/Algorithm/          # QuaternionEKF, Kalman, LPF, Ramp
├── Components/Controller/         # PID
├── Components/Device/             # DM8009, M3508, BMI088, Remote (SBUS)
├── BSP/                           # HAL wrappers: CAN, SPI, UART, ADC, PWM
├── Core/, Drivers/                # STM32 HAL + startup
├── simulation_core/               # C++17 library + pybind11 bindings (branch sim-core-cpp)
├── tools/                         # Python LQR computation (v3.1)
├── get_K.m, Balance_Luntuimatlab.m, robot_params.m  # MATLAB offline parameter generation (v3.1)
└── .clinerules                    # Task constraints for AI-assisted refactoring
```

---

## 5. Recent Architectural Changes

| PR | Scope | Rationale |
|----|-------|-----------|
| #13 | I/O boundary isolation | Eliminated data race: `CAN_Task` no longer reads `Control_Info.SendValue` directly; `motor_active` flag added to `motor_command_packet_t` |
| #14 | LQR gain evaluation | Replaced 72 `powf()` calls/cycle with Horner's method (monomial precomputation) |
| #15 | Dead code removal | Stripped 104 lines of commented-out legacy LQR-K parameter sets and torque conversion variants |
| #16 | Stack-local intermediates | Moved 8 geometric temporaries from `Leg_Info_Typedef` to `VMC_Calculate()` stack frame; `.bss` reduced by ~112 bytes |
| #17 | Module decomposition | Split 1238-line `Control_Task.c` into 5 domain modules + 105-line orchestrator |
| #18 | Safety & hygiene | Added `arm_sqrt_f32` zero-guard; deduplicated IMU reads; PID parameter macro; `_Static_assert` on snapshot size; `.gitignore` Keil artifacts |

Full analysis: `MDK-ARM/architecture_analysis_report.md`.

---

## 6. Debugging

- **VOFA+ telemetry**: `Control_Task` loop transmits 4 × float32 via UART (JustFloat protocol). Default channels: Target_Velocity, R/L chassis velocity, L chassis position.
- **SEGGER SystemView**: FreeRTOS task timeline and CPU load profiling via RTT.

---

## 7. Contributing

```
main ← stable (compile + flash verified)
├── feature/<name>    — new capability / refactor
├── bugfix/<name>     — defect correction
├── chore/<name>      — documentation, tooling, cleanup
└── experiment/<name> — exploratory, may not merge
```

Commit convention: `type: summary`, where type ∈ {feat, fix, refactor, docs, chore, perf, test}.

No direct commits to `main`. All changes via Pull Request with review.

---

## 8. Critical Notes

- **Motor IDs**: FDCAN2 → DM8009 ×4 (IDs 0–3). FDCAN3 → M3508 ×2 (IDs 0x201–0x202).
- **Remote switch**: s[1] = 3 (init+balance), 1 (high leg), 2 (shutdown), 0 (shutdown).
- **Battery threshold**: VDC < 22 V triggers low-voltage condition.
- **New `.c` files**: must be manually added to Keil/EIDE project after module addition.
- **Control frequency**: The 1kHz loop is hard-real-time. Any computation exceeding 1ms will cause period jitter via `osDelayUntil`.
