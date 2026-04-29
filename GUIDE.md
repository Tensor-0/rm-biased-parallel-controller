# 🐣 轮腿机器人从零入门指南

> 本文面向**第一次接触本项目**的成员。不要求你懂控制理论——只要你愿意理解"这台机器为什么能站起来"，这篇文章就是写给你的。

---

## 一、先认识这台机器 —— 系统设计思想

### 1.1 它长什么样？

想象一辆独轮车，但它有两条腿和两个轮子：

```
         ┌─────────────────┐
         │   🧠 主控板      │  ← STM32H723, 塞在机身里
         │   📡 BMI088 IMU  │  ← 感觉"倾斜"的传感器
         └────────┬────────┘
        ┌────────┴────────┐
        │                 │
   ┌────┴────┐       ┌────┴────┐
   │ 左腿 🦿  │       │ 右腿 🦿  │
   │ 大腿电机  │       │ 大腿电机  │  ← 4× DM8009 大喵电机
   │ 小腿电机  │       │ 小腿电机  │
   │   ⚙️轮子   │       │   ⚙️轮子   │  ← 2× M3508 轮毂电机
   └─────────┘       └─────────┘
```

### 1.2 核心设计哲学：分而治之

> **大问题无法一次性解决？那就把它拆成层级分明的小问题。**

我们的代码遵循经典的**嵌入式四层架构**：

| 层级 | 比喻 | 实际内容 | 例子 |
|------|------|---------|------|
| **Task 任务调度层** | 大脑皮层，做决策 | 4 个 FreeRTOS 线程 | "现在该平衡？还是该前进？" |
| **Algorithm 算法层** | 小脑，自动反应 | PID, LQR, 滤波器 | "倾斜 3° 应该用 5N 的力推回去" |
| **Device 设备驱动层** | 神经末梢，收发信号 | 电机/IMU/遥控器协议解析 | "电机当前位置是 -0.23 弧度" |
| **BSP 硬件抽象层** | 骨骼肌肉，物理执行 | HAL 库二次封装 | "向 CAN 总线发送这 8 个字节" |

**为什么要这样分层？** 想象你要换一颗更高级的 IMU 芯片——你只需要修改 Device 层的一个文件，Task 层的所有代码完全不需要动。这就是"解耦"的好处。

### 1.3 数据是怎么流动的？

把数据想象成在一条流水线上传送：

```
时刻 T=0ms:
  🏃 传感器报告:       "倾角=2.3°, 左膝=15°, 遥控器推了前进杆"
         │
         ▼
  📦 打包成快照:        control_input_snapshot_t (一瞬间拍下所有传感器状态)
         │
         ▼
  🧮 控制算法计算:      "根据当前状态, 左大腿应该输出 3.2 N·m 的扭矩"
         │
         ▼
  📦 打包成命令:        g_motor_cmd (一瞬间准备好所有电机指令)
         │
         ▼
  🚀 下发到电机:        CAN 总线上发出 8 字节数据帧

时刻 T=1ms: 重复上述过程 (1000 次/秒！)
```

> ⚡ **关键设计**: Task 层只通过 `control_input_snapshot_t` 读传感器，只通过 `g_motor_cmd` 写电机指令。Task 层**永远不直接碰**硬件全局变量。这就像餐厅的前台(算法)不会直接进后厨(硬件)翻冰箱，而是通过菜单(快照)和出菜口(命令包)。

---

## 二、它怎么站稳的？—— 运动控制设计思想

### 2.1 从"独轮车"说起

先忘掉两条腿。想象你骑着一辆独轮车：

- 你向前倾 → 独轮车会加速向前，把你"接住"
- 你向后仰 → 独轮车减速，让你恢复直立

这就是**平衡的本质**：**检测倾斜 → 计算需要的力 → 移动轮子来抵消倾斜**。

### 2.2 VMC (虚拟模型控制)：把两条腿简化成一根棍子

真实的腿有两个关节（大腿 + 小腿），非常复杂。但经过数学变换，它等效于：

```
真实腿:                         虚拟腿:
  ┌─● 大腿关节                     │
  │  ╲                            │
  │   ╲ 小腿连杆                   │  L₀ (虚拟腿长)
  │    ╲                          │
  │     ● 膝盖                    │
  │    ╱                          │
  │   ╱ 小腿连杆                   │
  │  ╱                            │
  └─● 脚 (轮子)                  ● 脚 (轮子)
```

> 🔑 **核心思想**: 不管大腿小腿怎么弯曲，我们只关心"从髋关节到轮子中心的长度 L₀"和"这条虚拟腿的倾斜角度 φ₀"。把两条复杂的连杆变成一根简单的棍子，控制算法就简单了 100 倍。

### 2.3 LQR (线性二次调节器)：自动计算最优推力

有了"虚拟腿"，问题变成了：**虚拟腿倾斜了，轮子该用多大力？**

LQR 就是解决这个问题的数学工具。你不需要理解矩阵运算，只需要知道：

```
LQR 就像一个"自动修车师傅":

状态 (X):           "车现在倾了 5°, 速度 0.2m/s, 位置偏了 3cm..."
      │
      ▼
LQR 查表 (K):       "在腿长 0.18m 的情况下，K 矩阵是 [K₁,K₂,K₃,K₄,K₅,K₆]"
      │
      ▼
输出 (u):           "你应该: 轮子输出 4.7 N·m, 关节输出 -1.2 N·m"
```

**增益调度**：腿越长，杠杆效应越大，需要的力越小。所以 K 矩阵会根据腿长自动变化：

```
K(L₀) = K₀ × L₀³ + K₁ × L₀² + K₂ × L₀ + K₃
```

这 12 组多项式系数 (K11~K26) 是提前用 MATLAB 算好的，直接写在代码里。

### 2.4 PID (比例-积分-微分)：简单可靠的"老黄牛"

LQR 负责"站得稳"，PID 负责"走得准"：

| PID 控制器 | 负责什么 | 通俗理解 |
|-----------|---------|---------|
| `PID_Leg_length_F[2]` | 左右腿长控制 | "我要腿长 0.20m，实际腿长 0.19m，用力推！" |
| `PID_Yaw[2]` | 偏航转向控制 | "我要转 30°，现在只转了 20°，继续转！" |
| `PID_Leg_Roll_F` | 横滚补偿 | "机身向左歪了，右腿多撑一点力！" |
| `PID_Leg_Coordinate` | 防劈叉 | "两条腿角度差太大，同步一下！" |

### 2.5 离地检测：机器人怎么知道自己在空中？

这是一个精巧的设计：

```
1. 从关节电机的力矩反馈，反算出地面给轮子的支持力 FN
2. 如果 FN < 22N → 这条腿在空中！
3. 空中的腿: 禁用 LQR 的轮子力矩输出（蹬不到地，转也白转）
4. 着地的腿: 加大重力补偿，单独支撑整个机身
```

### 2.6 控制流水线全景图

每个 1ms 周期，机器人按以下顺序执行 11 步：

```
1  📊 采集快照     ── 所有传感器瞬间拍照
2  🔋 低电压检测   ── 电池电压 < 22V 就报警
3  🔄 模式更新     ── 遥控器开关决定是平衡/关机/初始化
4  🔗 关节角度映射 ── 电机角度 → 右腿右膝的摆角
5  📐 VMC 正运动学 ── 两个角度 → 一根虚拟棍子(L₀, φ₀)
6  📈 LQR 增益更新 ── 根据新腿长查多项式得到 K 矩阵
7  🧲 传感器融合   ── IMU + 轮速计 → 6 维状态向量
8  🚗 底盘控制     ── 前进/后退/转向/高度/横滚 五个通道
9  👣 支撑力计算   ── 检测哪条腿着地
10 🔢 LQR 输出      ── u = -K·X 算出轮子力矩 T 和关节扭矩 Tp
11 ⚡ 关节力矩映射 ── 雅可比矩阵 → 4 个关节电机的真实扭矩
```

---

## 三、上手流程 —— 从拉代码到跑起来

### 3.1 环境准备 (30 分钟)

**你需要**：
- 一台 Windows 电脑 (主力) 或 Ubuntu 22.04 (进阶)
- J-Link / ST-Link 调试器
- 遥控器 (DJI 原厂)
- 已经组装好的轮腿机器人

**软件安装**：

```bash
# ===== Linux 环境 =====
# 1. 安装 ARM GCC 编译器
sudo apt-get install arm-none-eabi-gcc

# 2. VS Code 中安装 EIDE 插件
#    搜索 "Embedded IDE" → 安装

# ===== Windows 环境 =====
# 1. 安装 Keil MDK v5.38+
# 2. 安装 STM32H7xx Packs
#    (在 Keil 中: Pack Installer → 搜索 STM32H7 → Install)
```

### 3.2 拉代码 & 编译 (10 分钟)

```bash
# 1. 克隆仓库
git clone https://github.com/Tensor-0/rm-biased-parallel-controller.git
cd rm-biased-parallel-controller

# 2. 切换到最新稳定版
git checkout main
git pull origin main

# 3. 打开工程
#    Keil:  双击 MDK-ARM/COD_H7_Template.uvprojx
#    VS Code: 打开根目录 → EIDE 自动识别

# 4. 编译
#    Keil: F7
#    VS Code: 点击 EIDE 面板的 🔨 按钮

# 期望输出: "0 Error(s), 0 Warning(s)"
```

### 3.3 烧录到开发板 (5 分钟)

```bash
# 方式 1: Keil 内烧录
#  1. 连接 J-Link/ST-Link
#  2. 点击 Flash → Download (F8)

# 方式 2: 命令行烧录 (Linux)
openocd -f board/stm32h723discovery.cfg \
        -c "program build/COD_H7_Template.hex verify reset exit"
```

### 3.4 第一次上电 (重要！)

> ⚠ **首次上电前检查**:
> 1. 所有电机已牢固安装
> 2. BMI088 IMU 方向正确（平放时机身 Pitch 角应为 0°）
> 3. 遥控器电池充足
> 4. 周围有至少 2m × 2m 的空地

**上电流程**：

```
步骤 1: 打开机器人电源
步骤 2: 等待 3 秒 (电机使能)
步骤 3: 将遥控器开关 s[1] 拨到 ↓ (位置 3)
       → 四个关节电机会自动归位到安全角度
       → LED 灯会闪烁表示初始化中
步骤 4: 等待关节归位完成 (约 2 秒)
       → 机器人自动进入平衡模式
       → 此时可以推遥控器摇杆控制前进后退
步骤 5: 测试完毕 → 开关 s[1] 拨到位置 2 → 所有电机关闭
```

### 3.5 如何修改参数

假设你想改"平衡的力度"（LQR 参数），不需要重新编译整个工程：

```c
// 文件: Application/Task/Src/lqr_controller.c
// 找到这段 (大约第 18 行):

float K11[6] = {0, -344.130023f,  397.724995f,  -265.059481f,  -4.941964f};
float K12[6] = {0,  11.842778f,  -18.891159f,  -27.922778f,    0.234829f};
// ... 共 12 行

// Kij[m] = Kij[1]×L₀³ + Kij[2]×L₀² + Kij[3]×L₀ + Kij[4]
// 改 Kij[1:4] 的值 → 重新编译 → 烧录 → 测试
```

> 💡 **提示**: 用 MATLAB 脚本 `get_K.m` 可以自动计算新的 K 矩阵多项式系数。

---

## 四、深入理解 —— 代码导读

### 4.1 阅读顺序建议

| 顺序 | 文件 | 为什么先读它 |
|------|------|-------------|
| 🥇 | [`README.md`](README.md) | 项目总览 |
| 🥈 | [`Control_Task.h`](Application/Task/Inc/Control_Task.h) | 看懂核心数据结构 |
| 🥉 | [`Control_Task.c`](Application/Task/Src/Control_Task.c) | 看编排器怎么调度 (只有 100 行) |
| 4 | [`mode_state_machine.c`](Application/Task/Src/mode_state_machine.c) | 初始化流程 + PID 参数 |
| 5 | [`vmc_kinematics.c`](Application/Task/Src/vmc_kinematics.c) | 核心正运动学公式 |
| 6 | [`lqr_controller.c`](Application/Task/Src/lqr_controller.c) | LQR 增益调度 |
| 7 | [`sensor_fusion.c`](Application/Task/Src/sensor_fusion.c) | 传感器融合 |
| 8 | [`chassis_control.c`](Application/Task/Src/chassis_control.c) | 底盘控制 |
| 9 | [`CAN_Task.c`](Application/Task/Src/CAN_Task.c) | CAN 通信 |
| 10 | [`INS_Task.c`](Application/Task/Src/INS_Task.c) | IMU 姿态解算 |

### 4.2 关键结构体速查

```c
// 控制算法核心: Control_Info_Typedef
// 定义在: Application/Task/Inc/Control_Task.h

typedef struct {
    Chassis_Situation_e Chassis_Situation; // 当前状态: WEAK 或 BALANCE
    Leg_Info_Typedef L_Leg_Info;          // 左腿全部信息 (500+ 字节)
    Leg_Info_Typedef R_Leg_Info;          // 右腿全部信息
    float Target_Velocity;                // 目标底盘速度 (m/s)
    float Chassis_Velocity;               // 实际底盘速度
    float Accel;                          // 底盘线加速度
    float Yaw_Err;                        // 偏航角度误差
    Init_Struct Init;                     // 初始化状态机
    Roll_Struct Roll;                     // 横滚控制参数
} Control_Info_Typedef;
```

### 4.3 修改代码的"黄金法则"

> 🔴 **绝不在 main 分支上直接修改代码！**

```bash
# 正确流程:
git checkout main && git pull                  # 1. 确保是最新
git checkout -b feature/my-change              # 2. 创建分支
# ... 改代码 ...
git add . && git commit -m "feat: 我的改动"    # 3. 提交
git push -u origin feature/my-change           # 4. 推送
# ... 在 GitHub 上创建 Pull Request ...         # 5. Code Review
```

---

## 五、常见问题 FAQ

### Q1: 编译报错 "undefined reference to xxx" 怎么办？

A: 新添加的 `.c` 文件需要在 Keil 工程/EIDE 配置中手动添加。

### Q2: 机器人一上电就疯狂抖动？

A: 检查 IMU 的安装方向。`INS_Task.c` 中的 `Angle[2]` 读出来应该是机身倾斜角。如果方向反了，改 `sensor_fusion.c` 第 17 行的负号。

### Q3: 遥控器不响应？

A: 检查遥控器是否已对频。确认 USART 接收到的 SBUS 数据帧完整（18 字节）。

### Q4: 想添加新的控制功能（如跳跃）？

A: 
1. 在 `chassis_control.h` 中声明新函数
2. 在 `chassis_control.c` 中实现
3. 在 `Control_Task.c` 的编排器循环中调用
4. 创建 PR 等待 Review

### Q5: 看不懂 LQR 矩阵运算？

A: 不需要看懂。这些矩阵是用 MATLAB 离线计算的。你只需要知道 `u = -K·X` 这个公式的意义：**K 告诉机器人"倾斜多少度应该用多少力"，X 告诉机器人"你现在差目标状态多少"**。

---

## 📚 延伸阅读

| 文档 | 适合什么阶段 |
|------|-------------|
| [`README.md`](README.md) | 刚加入项目，了解概况 |
| **本文** (`GUIDE.md`) | 开始看代码前，理解设计思想 |
| [`architecture_analysis_report.md`](MDK-ARM/architecture_analysis_report.md) | 准备做代码贡献，了解优化方向 |
| [`architecture_refactor_plan.md`](architecture_refactor_plan.md) | 参与重构，分解任务 |
| [GitHub Issues](https://github.com/Tensor-0/rm-biased-parallel-controller/issues) | 找活干：P0=紧急, P1=重要, P2=改善, P3=锦上添花 |

---

> 💬 **遇到问题？** 在 [GitHub Issues](https://github.com/Tensor-0/rm-biased-parallel-controller/issues) 中提问，附上你的现象描述 + 控制台日志。

> 🎯 **记住**: 这个项目是用 1000Hz (每秒 1000 次) 的速度在控制一台机器人站稳。每 1 毫秒，它完成 11 步计算、发出 4 个电机指令。这是实时系统的魔法——在你感觉不到的瞬间，它已经完成了所有思考。
