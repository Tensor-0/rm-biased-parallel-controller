# MATLAB Offline Computation — LQR Gain Coefficient Generation

> **Target**: Engineers modifying physical robot parameters. Document covers: three-body dynamics derivation, symbolic linearization, polynomial fitting pipeline, Ubuntu-compatible alternatives.

---

## 1. File Inventory

| File | Type | Role |
|------|------|------|
| `robot_params.m` | Configuration | Mechanical parameters, Q/R weights, leg sampling range, LQR method switch |
| `Balance_Luntuimatlab.m` | Core function | Builds 3-body dynamics, performs symbolic Jacobian linearization, calls `lqr()` |
| `get_K.m` | Orchestration script | Iterates 31 leg lengths, collects K matrices, fits cubic polynomials, outputs C arrays |
| `Balance_Luntuimatlab.asv` | Editor backup | Legacy version using discrete `dlqr()`; retained for reference |
| `tools/lqr_gain_schedule.py` | Python alternative | Pure-numeric LQR via `scipy.linalg.solve_continuous_are()`; no MATLAB license required |

---

## 2. Three-Body Dynamics Model

```
Body (M=18.12 kg) ─┐
                   ├─ Hip joint (Tp)
Leg (mp=1.12 kg) ──┤
                   ├─ Wheel axle
Wheel (mw=0.88 kg) ─┘
```

### 2.1 Newton-Euler Equations

**Equation 1** — Wheel horizontal motion:
```
d²x = (T − N·R) / (Iw/R + mw·R)
```

**Equation 2** — Leg rotation about wheel axle:
```
Ip·d²θ = (P·L + Pm·Lm)·sin(θ) − (N·L + Nm·Lm)·cos(θ) − T + Tp
```

**Equation 3** — Body rotation about hip:
```
Im·d²φ = Tp + Nm·l·cos(φ) + Pm·l·sin(φ)
```

where N, Nm are horizontal reaction forces; P, Pm are vertical reaction forces (including gravity). All variables are symbolic until numerical substitution at the linearization step.

### 2.2 State-Space Linearization

At equilibrium (all states = 0, inputs = 0), the Jacobian yields A (6×6) and B (6×2):

```
State: x = [θ, dθ, x, dx, φ, dφ]
Input: u = [T, Tp]
```

MATLAB `jacobian()` computes symbolic partial derivatives; `subs()` substitutes equilibrium conditions; `double()` converts to numeric matrices suitable for `lqr()`.

---

## 3. LQR Weight Matrices

```matlab
Q = diag([30, 1, 500, 100, 5000, 1]);   % [θ dθ x dx φ dφ]
R = diag([1, 0.25]);                       % [T Tp]
```

| Q index | State | Weight | Engineering rationale |
|---------|-------|--------|----------------------|
| Q(1,1) | θ (leg inclination) | 30 | Moderate penalty — leg angle is indirectly controlled via body |
| Q(3,3) | x (chassis position) | 500 | Position drift prevention |
| Q(5,5) | φ (body inclination) | 5000 | **Dominant** — body tilt is the primary instability |
| R(1,1) | T (wheel torque) | 1 | Baseline cost |
| R(2,2) | Tp (hip torque) | 0.25 | Hip is 4× cheaper → LQR prefers hip over wheel for balance |

**Method**: Continuous `lqr(A, B, Q, R)` solving the algebraic Riccati equation `AᵀP + PA − PBR⁻¹BᵀP + Q = 0`, yielding optimal gain K = R⁻¹BᵀP.

---

## 4. Gain Scheduling Pipeline (`get_K.m`)

```
1. Load robot_params.m          → read M, mw, mp, Q, R, leg range
2. for leg = 0.10:0.01:0.40     → 31 samples
     k = Balance_Luntuimatlab(leg)
     collect k11..k26
3. polyfit(leg, kij, 3)          → cubic polynomial coefficients
4. Display RMSE per Kij          → fitting quality indicator
5. fprintf C array literals      → copy-paste into lqr_controller.c
6. Plot 12 subplots              → data points vs. fitted curve
```

**Output**: 12 lines of the form:
```c
float K11[6] = {0, -344.130023f, 397.724995f, -265.059481f, -4.941964f};
```

where `Kij[1]` through `Kij[4]` are the cubic (L₀³), quadratic (L₀²), linear (L₀), and constant terms respectively. `Kij[0]` is reserved.

---

## 5. C Code Integration

At runtime, `LQR_K_Update()` in `lqr_controller.c` evaluates:

```c
float L_L0_3 = L_L0 * L_L0 * L_L0;
K[0][0] = K11[1]*L_L0_3 + K11[2]*L_L0*L_L0 + K11[3]*L_L0 + K11[4];
```

12 coefficients × 2 legs = 24 evaluations per 1kHz cycle.

---

## 6. Workflow When Modifying Physical Parameters

1. Edit `robot_params.m`: update masses, link lengths, Q/R weights
2. Run `get_K` in MATLAB, or `python3 tools/lqr_gain_schedule.py` (pure Python)
3. Copy the 12 generated `float Kij[6]` lines into `lqr_controller.c`
4. Verify maximum RMSE < 1.0 (printed by both MATLAB and Python scripts)
5. If RMSE > 1.0: increase `poly_order` from 3 to 4, or reduce leg range

---

## 7. Ubuntu 22.04 Alternatives (No MATLAB License)

### Option A: Python (recommended)
```bash
pip install numpy scipy matplotlib
python3 tools/lqr_gain_schedule.py
```
Uses `scipy.linalg.solve_continuous_are()` for the Riccati equation. Output format identical to MATLAB version.

### Option B: GNU Octave
```bash
sudo apt install octave octave-control octave-symbolic
octave --eval "run('get_K.m')"
```
Requires `octave-symbolic` for `syms` support.

### Option C: Use existing coefficients
K coefficients in `lqr_controller.c` are valid for body mass 15–22 kg, leg length 0.12–0.22 m, wheel radius 0.05–0.06 m. LQR feedback is inherently robust to ±10% parameter mismatch.

---

## 8. Known Issues

| Issue | Resolution |
|-------|-----------|
| `dlqr()` vs `lqr()` discrepancy | Current code uses continuous `lqr()`. Legacy `.asv` backup used discrete `dlqr()` with `Ts=0.001s`; at 1kHz the difference is < 1% |
| Numerical conditioning at leg → 0 | `S_Radicand` can approach zero; `if(S < 1e-6) S = 1e-6` guard added in PR #18 |
