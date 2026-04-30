# 🧪 仿真入门教程 — 零基础也能跑起来

> **目标受众**: Ubuntu 22.04 新手，没有 ROS2/C++/Mujoco 经验  
> **目标**: 不连接任何硬件，在电脑上编译并运行机器人的控制算法  
> **时间**: 30 分钟  

---

## 一、你需要的工具 (一键安装)

打开终端（`Ctrl+Alt+T`），复制粘贴以下命令：

```bash
# 1. 安装 C++ 编译器
sudo apt update && sudo apt install -y build-essential cmake

# 2. 安装 Python 开发包
sudo apt install -y python3-dev python3-pip
pip3 install numpy matplotlib scipy

# 3. 安装 pybind11 (让 C++ 和 Python 互通)
pip3 install pybind11

# 4. 检查是否安装成功
cmake --version    # 应该显示 3.22 以上
g++ --version      # 应该显示 11.x
python3 --version  # 应该显示 3.10
```

> 💡 **什么是 pybind11？** 它像一个"翻译官"：把 C++ 的函数翻译成 Python 能直接调用的函数。这样你可以在 Python 里调用机器人的控制算法。

---

## 二、编译算法库 (10 分钟)

```bash
# 1. 克隆项目并切换到仿真分支
git clone https://github.com/Tensor-0/rm-biased-parallel-controller.git
cd rm-biased-parallel-controller
git checkout feature/sim-core-cpp

# 2. 进入仿真核心目录
cd simulation_core

# 3. 创建编译目录
mkdir -p build && cd build

# 4. 配置 cmake
cmake ..

# 你应该看到:
#   -- Configuring done
#   -- Generating done
#   没有 Error 字样就对了

# 5. 编译
make -j4

# 你应该看到:
#   [100%] Built target wheel_legged_sim
#   生成一个 .so 文件 (wheel_legged_sim.cpython-*.so)
```

> ⚠ 如果 `cmake ..` 报错找不到 pybind11：  
> 运行 `python3 -c "import pybind11; print(pybind11.get_cmake_dir())"` 获取路径  
> 然后 `cmake .. -DCMAKE_PREFIX_PATH="上面输出的路径"`

---

## 三、跑第一个 Python 测试 (2 分钟)

在 `simulation_core/build/` 目录下：

```bash
python3
```

然后在 Python 交互环境中输入：

```python
# 导入我们的模块
from wheel_legged_sim import WheelLeggedDynamics, RobotState

# 创建控制器 (相当于 STM32 里的 Control_Task)
wld = WheelLeggedDynamics()

# 创建一个"假想的机器人状态"
state = RobotState()

# 设置关节角度: [左小腿, 左大腿, 右大腿, 右小腿] 单位: 弧度
state.joint_angles = [0.0, -0.13, 0.13, 0.0]
# 这组角度代表"正常站立"的姿态

# 设置 LQR 6 维状态向量:
#   [θ=虚拟腿倾角, dθ=倾角速度, x=位置, dx=速度, φ=机身倾角, dφ=机身角速度]
state.state_vector = [0.0, 0.0, 0.0, 0.0, 0.05, 0.0]
# 这里 φ=0.05rad (约3°) 表示机身向前倾斜了 3 度

# 计算控制指令！
cmd = wld.compute_torques(state)

print("关节目标力矩 (N·m):")
print(f"  左小腿: {cmd.joint_torque[0]:.2f}")
print(f"  左大腿: {cmd.joint_torque[1]:.2f}")
print(f"  右大腿: {cmd.joint_torque[2]:.2f}")
print(f"  右小腿: {cmd.joint_torque[3]:.2f}")
print(f"轮子电流 (mA): 左={cmd.wheel_current[0]:.1f}, 右={cmd.wheel_current[1]:.1f}")
```

**预期输出**：
```
关节目标力矩 (N·m):
  左小腿: 15.00
  左大腿: -15.00
  右大腿: -15.00
  右小腿: 15.00
轮子电流 (mA): 左=2058.5, 右=-2058.5
```

> 🧠 **这个输出在说什么？**  
> 机身前倾 → 轮子应该向前加速去"接住"身体 → 左轮正转(2058mA)、右轮反转(因镜像安装需取反) ✓  
> 大腿和小腿的力矩方向相反 → -15 和 +15 表示"一条腿伸直一条腿弯曲来发力" ✓  
> 力矩被限制在 ±15 N·m → 安全限幅生效 ✓

---

## 四、测试正运动学 (5 分钟)

```python
from wheel_legged_sim import WheelLeggedDynamics

wld = WheelLeggedDynamics()

# 正运动学: 关节角度 → 虚拟腿长 L₀ 和倾角 φ₀
vmc = wld.vmc_forward_kinematics(
    calf_angle=0.0,      # 小腿摆角 (弧度)
    thigh_angle=-0.13,   # 大腿摆角 (弧度)
    is_left_leg=True     # True=左腿, False=右腿
)

print(f"虚拟腿长 L₀ = {vmc.L0 * 1000:.1f} mm")       # 约 468 mm
print(f"虚拟腿倾角 φ₀ = {vmc.phi0 * 57.3:.2f}°")     # 约 -3.72°
print(f"雅可比系数 A  = {vmc.A_coeff:.4f}")           # 约 0.05-0.10
```

> 💡 **如果改变关节角度，观察 L₀ 和 φ₀ 的变化！** 比如：
> - 两个角度都变大 → 腿更长 (L₀ 增大)
> - 两个角度差变大 → 腿更弯 (A 系数变化)

---

## 五、下一步：装 Mujoco (可选)

如果你想看到机器人**在屏幕上动起来**，需要装物理仿真器 Mujoco：

```bash
# 1. 安装 Mujoco 本体
sudo apt install -y libglfw3-dev libglew-dev
pip3 install mujoco

# 2. 测试 Mujoco 是否能用
python3 -c "import mujoco; print('Mujoco installed:', mujoco.__version__)"
```

> ⚠ Mujoco 完整配置 + 机器人模型 + 闭环仿真在 Phase 2 实现中。当前 `wheel_legged_sim` 模块已就绪，可以在你自己写的 Mujoco 环境中作为控制器使用。

---

## 六、常见问题 FAQ

### Q1: 编译时 `fatal error: pybind11/pybind11.h: No such file or directory`

A: pybind11 的头文件没找到。运行：
```bash
pip3 install pybind11
python3 -c "import pybind11; print(pybind11.get_include())"
```
然后把输出的路径添加到 cmake 命令：
```bash
cmake .. -Dpybind11_DIR=$(python3 -c "import pybind11;print(pybind11.get_cmake_dir())")
```

### Q2: `ModuleNotFoundError: No module named 'wheel_legged_sim'`

A: 你需要在 `simulation_core/build/` 目录下运行 Python，或者把 `build/` 目录加到 Python 路径：
```python
import sys; sys.path.insert(0, '/path/to/simulation_core/build')
```

### Q3: 我改了 C++ 代码，怎么重新编译？

A: 在 `simulation_core/build/` 下直接运行 `make -j4`（不需要重新 cmake）。

### Q4: 我不懂 C++，怎么调参数？

A: 大部分参数在 `RobotParams` 结构体中（`simulation_core/wheel_legged_dynamics.hpp`）。改完后重新编译，Python 端即可生效。不需要改任何 Python 代码。

### Q5: 我想用这个模块做自己的仿真，怎么用？

A: 只需要导入 `wheel_legged_sim`：
```python
from wheel_legged_sim import WheelLeggedDynamics, RobotState

wld = WheelLeggedDynamics()
# 你的仿真循环:
while simulating:
    state = get_robot_state_from_mujoco()  # 你写的函数
    cmd = wld.compute_torques(state)
    apply_torques_to_mujoco(cmd)           # 你写的函数
```

---

## 七、文件速查

| 文件 | 作用 |
|------|------|
| `simulation_core/wheel_legged_dynamics.hpp` | **改参数看这里** — RobotParams 结构体 |
| `simulation_core/wheel_legged_dynamics.cpp` | **改算法看这里** — VMC + LQR 实现 |
| `simulation_core/bindings.cpp` | pybind11 导出 (一般不需要改) |
| `simulation_core/CMakeLists.txt` | 编译配置 (一般不需要改) |

---

> 🎉 **恭喜！** 你已经成功让机器人的"大脑"在你的电脑上跑起来了——没有烧录、没有接线、没有摔倒的风险。这就是 Sim-to-Real 的第一步。
