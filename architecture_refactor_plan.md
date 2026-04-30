
---

# 14. Sim-Core 提取 — C++ 算法库 (Phase 1, 2025.04)

> **状态**: 🚧 已完成, 待集成仿真

## 14.1 动机

原 STM32 算法代码 (VMC 正运动学 + LQR 增益调度) 与硬件深度耦合。不修改物理参数就无法"无下游"验证算法的正确性。Sim-Core 提取的目标:

1. **彻底剥离硬件**: 移除 HAL, FreeRTOS, CAN, SPI, EKF, 传感器融合, 遥控器 — 只留纯数学
2. **现代 C++ 封装**: 全局变量 → 类成员, C 数组 → `std::array`, 结构体 → 类方法
3. **pybind11 导出**: C++ → Python 模块, 供 Mujoco / PyBullet / 纯 Python 仿真直接调用

## 14.2 新增文件

| 文件 | 说明 |
|------|------|
| `.clinerules` | ROO / Claude 任务约束: 禁止 HAL / FreeRTOS / EKF |
| `simulation_core/wheel_legged_dynamics.hpp` | `WheelLeggedDynamics` 类 + 5 个数据类型 (`RobotParams`, `VmcResult`, `LqrResult`, `MotorCommand`, `RobotState`) + `K_COEFFS` 常数 |
| `simulation_core/wheel_legged_dynamics.cpp` | 完整算法实现: `vmc_forward_kinematics`, `lqr_gain_schedule`, `jacobian_torque_mapping`, `compute_torques` |
| `simulation_core/bindings.cpp` | pybind11 绑定 → Python 模块 `wheel_legged_sim` |
| `simulation_core/CMakeLists.txt` | C++17 构建, 自动检测 pybind11 (system / anaconda3) |

## 14.3 C++ ↔ C 映射

| STM32 C 函数 | C++ 方法 | 变化 |
|-------------|---------|------|
| `VMC_Calculate()` | `vmc_forward_kinematics()` | 全局 `Control_Info` → 参数传入/返回 `VmcResult` |
| `LQR_K_Update()` + `LQR_T_Tp_Calculate()` | `lqr_gain_schedule()` | 合并为一步: 霍纳 → `K[]` → 矩阵乘 `K·X` |
| `VMC_Joint_Tourgue_Calculate()` | `jacobian_torque_mapping()` | 限幅逻辑内嵌 |
| `VMC_Measure_F_Tp_Calculate()` | `inverse_dynamics()` + `contact_detection()` | 分为独立方法, 可单独测试 |
| 无 | `compute_torques()` | 全流水线入口: 关节角度 + 6-D 状态 → 6 个电机指令 |

## 14.4 构建与测试

```bash
cd simulation_core
mkdir -p build && cd build
cmake .. && make -j4

python3 -c "
from wheel_legged_sim import WheelLeggedDynamics, RobotState
wld = WheelLeggedDynamics()
state = RobotState()
state.state_vector = [0,0,0,0,0.05,0]   # 机身倾斜 3°
state.joint_angles  = [0,-0.13,0.13,0]   # 正常站立
cmd = wld.compute_torques(state)
print(f'joint torque = {cmd.joint_torque}')
print(f'wheel current = {cmd.wheel_current}')
"
```

**验证结果** (机身倾斜 0.05rad):
- 关节力矩: [15.0, -15.0, -15.0, 15.0] N·m ✓
- 轮子电流: [2058.5, -2058.5] mA ✓
- 虚拟腿长 L₀: 468.1 mm, 倾角 φ₀: -3.72° ✓

## 14.5 下一步 (Phase 2)

- [ ] Mujoco 环境 (轮腿机器人模型 + `wheel_legged_sim` 作为控制器)
- [ ] 闭环仿真: 真值状态 → LQR → 力矩 → 物理 → 新状态
- [ ] 对比验证: Mujoco 中的 LQR 输出 vs 真实 STM32 数据
- [ ] 扩展: 底盘控制 (`Chassis_Control`) + 腿长 PID + 横滚补偿

