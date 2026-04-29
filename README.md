# rm-biased-parallel-controller

> **偏置并联轮腿机器人运动控制框架**  
> 基于 STM32H723 + FreeRTOS，面向 RoboMaster 竞赛的 C 语言实时控制系统

---

## 📖 项目简介（给初学者）

### 这个项目是做什么的？

本项目控制一台**双轮腿机器人**——它用两条机械腿和两个轮子来站立、行走和转弯。每条腿由 2 个大喵电机（DM8009）驱动，外加 1 个轮毂电机（M3508）。机体上安装 BMI088 惯性传感器来感知倾斜角度。

你需要知道的基本概念：

| 概念 | 通俗解释 |
|------|---------|
| **VMC（虚拟模型控制）** | 把复杂的连杆腿简化成一根"虚拟弹簧腿"，方便计算 |
| **LQR（线性二次调节器）** | 一种自动计算"用多大力气才能站稳"的算法 |
| **正运动学** | 知道关节角度 → 算出脚在哪里 |
| **雅可比矩阵** | 把"脚底该用多大力"翻译成"每个关节该输出多大扭矩" |
| **FreeRTOS** | 一个轻量级操作系统，让多个任务"看起来同时运行" |

### 系统架构一览

```
┌────────────────────────────────────────────────────────┐
│                    Task 调度层 (1kHz)                    │
│  INS_Task         Control_Task        CAN_Task          │
│  (姿态解算)  ────▶  (平衡控制)  ────▶  (电机通信)       │
├────────────────────────────────────────────────────────┤
│                  Algorithm 算法组件层                     │
│  PID · LQR · 四元数EKF · 低通滤波 · 斜坡函数              │
├────────────────────────────────────────────────────────┤
│                   Device 设备驱动层                       │
│  DM8009 · M3508 · BMI088 · SBUS遥控 · MiniPC图传         │
├────────────────────────────────────────────────────────┤
│                    BSP 硬件抽象层                         │
│  FDCAN · SPI · UART · ADC · PWM · GPIO                 │
├────────────────────────────────────────────────────────┤
│                  STM32H723VGTx (Cortex-M7, 550MHz)       │
└────────────────────────────────────────────────────────┘
```

### 数据流向

```
BMI088 ──SPI──▶ INS_Task ──全局变量──▶ control_input_snapshot_t (输入快照)
                                           │
遥控器 ──UART──▶ (中断)                      ▼
电机反馈──FDCAN──▶ (中断)            Control_Task (11步流水线)
                                           │
                                           ▼
                                    g_motor_cmd (输出命令包)
                                           │
                                           ▼
                                    CAN_Task ──FDCAN──▶ 电机
```

---

## 🚀 快速开始

### 硬件要求

- **主控**: STM32H723VGTx (RoboMaster A/B/C 型板卡均可)
- **电机**: 4× DM8009 关节电机 + 2× M3508 轮毂电机
- **传感器**: BMI088 六轴 IMU
- **遥控**: DJI 遥控器 (SBUS 协议)
- **供电**: 24V 电池

### 环境配置

我们同时支持两套开发环境：

#### 方式一：Keil MDK-ARM (Windows, 主力开发)

1. 安装 Keil MDK v5.38+
2. 安装 STM32H7xx 芯片包
3. 打开 `MDK-ARM/COD_H7_Template.uvprojx`
4. 点击 Build (F7)

#### 方式二：VS Code + GCC (Linux, 实验性)

1. 安装 `arm-none-eabi-gcc` (推荐 10.3.1+)
2. 在 VS Code 中安装 **EIDE** 插件
3. 打开项目根目录，EIDE 自动识别配置
4. 点击构建按钮

### 烧录测试

1. 使用 J-Link / ST-Link 连接开发板
2. 在 Keil 中点击 Download (F8) 或在终端运行 `openocd -f board/stm32h723discovery.cfg`
3. 上电后，将遥控器开关拨到位置 3（初始化模式）
4. 等待关节电机归位后自动进入平衡模式

---

## 📂 项目结构

```
rm-biased-parallel-controller/
├── Application/Task/
│   ├── Inc/
│   │   ├── Control_Task.h          # 全局控制状态结构体定义
│   │   ├── control_io.h            # I/O 边界定义 (输入快照 + 输出命令包)
│   │   ├── INS_Task.h              # IMU 姿态结构体定义
│   │   ├── CAN_Task.h / Detect_Task.h
│   │   ├── mode_state_machine.h    # 模式状态机模块
│   │   ├── vmc_kinematics.h        # VMC 运动学模块
│   │   ├── lqr_controller.h        # LQR 控制器模块
│   │   ├── chassis_control.h       # 底盘高层控制模块
│   │   └── sensor_fusion.h         # 传感器融合模块
│   ├── Src/
│   │   ├── Control_Task.c          # ★ 控制任务编排器 (~100 行)
│   │   ├── control_io.c            # I/O 边界实现
│   │   ├── INS_Task.c              # IMU 姿态解算任务
│   │   ├── CAN_Task.c              # CAN 通信任务
│   │   ├── mode_state_machine.c    # PID 初始化 + 模式状态机
│   │   ├── vmc_kinematics.c        # VMC 正运动学 + 力矩映射
│   │   ├── lqr_controller.c        # 增益调度 LQR
│   │   ├── chassis_control.c       # 底盘移动/高度/横滚控制
│   │   └── sensor_fusion.c         # IMU + 轮速计融合
├── Components/
│   ├── Algorithm/                   # 算法库 (四元数EKF, 卡尔曼滤波, 低通滤波)
│   ├── Controller/                  # PID 控制器
│   └── Device/                      # 设备驱动 (DM8009, M3508, BMI088, 遥控器)
├── BSP/                             # 底层驱动 (CAN, SPI, UART, ADC, PWM)
├── Core/                            # STM32 HAL 启动代码
├── Drivers/                         # CMSIS + HAL 库
├── MDK-ARM/                         # Keil 工程 + 架构分析报告
│   └── architecture_analysis_report.md  # ★ 详细的代码审查报告
├── architecture_refactor_plan.md    # 重构方案文档
├── .gitignore                       # Git 忽略规则
└── README.md                        # 本文件
```

---

## 🧠 核心算法详解

### 1. VMC 正运动学 (`vmc_kinematics.c`)

**目的**：把 2 个关节角度的测量值转换成一个"虚拟简化腿"的长度和角度。

```
真实结构:  Calf_Angle (θ₁)  ──┐
          Thigh_Angle (θ₂) ──┼── 连杆正运动学 ──▶ 虚拟腿长 L₀ + 摆角 φ₀
                              │
简化结构:                     ▼
                "一根棍子"的长度和倾角
```

关键公式（左腿）：
- `M = (θ₁ - θ₂) / 2`  — 夹角半差
- `N = (θ₁ + θ₂) / 2`  — 夹角半和
- `S = sqrt(b² - a²·sin²(M))`  — 几何中间量
- `L₀ = (a·cos(M) + S) / K`  — 虚拟腿长
- `φ₀ = N`  — 虚拟腿摆角

### 2. LQR 平衡控制 (`lqr_controller.c`)

**目的**：告诉机器人要用多大力才能保持平衡。

**6 维状态向量 X**（目标 - 测量）：
```
X = [
    θ_error,      // 虚拟腿倾角误差
    θ_dot_error,  // 虚腿角速度误差
    x_error,      // 位置误差
    v_error,      // 速度误差
    φ_error,      // 机身倾角误差
    φ_dot_error   // 机身角速度误差
]
```

**增益调度**：LQR 的 K 矩阵会根据当前虚拟腿长自动调整：
```
K(L₀) = K₀·L₀³ + K₁·L₀² + K₂·L₀ + K₃   (霍纳法则优化)
```

**控制输出**：
```
u = -K · X
其中 u[0] = T (轮子力矩), u[1] = Tp (关节扭矩)
```

### 3. 传感器融合 (`sensor_fusion.c`)

从多个传感器融合出可靠的 6 维状态向量：

```
IMU 角速度 ──▶ 姿态角 φ
IMU 加速度 ──▶ 底盘加速度 Accel
轮速 RPM   ──▶ 线速度 X ──▶ 预测速度 ──▶ 融合速度 (0.8×预测 + 0.2×测量)
融合速度   ──▶ 位置    (积分, 仅在定速模式)
```

### 4. 底盘控制 (`chassis_control.c`)

| 控制通道 | 遥控器输入 | 控制方法 |
|---------|-----------|---------|
| 前进/后退 | `ch[3]` 摇杆 | 斜坡函数 → 速度目标 |
| 左右转向 | `ch[2]` 摇杆 | 串级 PID (位置 + 速度) |
| 高度切换 | `s[1]` 开关 | 斜坡函数 → 腿长目标 |
| 横滚补偿 | IMU Roll 角 | 几何计算 + PID |

### 5. 模式状态机 (`mode_state_machine.c`)

```
遥控器 s[1]=3/1 ──▶ IF_Begin_Init = 1
       │
       ▼
  4 关节到位? ──▶ IF_Joint_Init = 1
       │
       ▼
  CHASSIS_BALANCE (平衡模式)
       │
       ▼  s[1]=2 或 0
  CHASSIS_WEAK   (虚弱/关机)
```

---

## 📝 架构重构历史 (V3.0, 2025.04)

本版本完成了全面的 Code Review 和模块化重构：

### PR #13 — I/O 解耦 (P0)
- `motor_command_packet_t` 新增 `motor_active` 标志
- CAN_Task 不再直接读取 `remote_ctrl` / `Control_Info` / `INS_Info`
- 移除 3 个不必要的 `#include`

### PR #14 — LQR 性能优化 (P3)
- `LQR_K_Update()` 中 72 次 `powf()`/周期 → 霍纳法则 (预计算 L₀², L₀³)

### PR #15 — 代码清理 (P2)
- 删除 104 行注释掉的历史 LQR-K 参数矩阵和扭矩转换代码

### PR #16 — 内存优化 (P1)
- `Leg_Info_Typedef` 中 8 个纯中间变量 (a,b,M,N,S,S_Radicand,t) 改为栈局部变量
- 减少 112 字节 .bss 段内存

### PR #17 — 模块化拆分 (P1)
- `Control_Task.c`: 1238 行 → 105 行编排器
- 拆分为 5 个独立模块:
  - `mode_state_machine.c` (92 行)
  - `vmc_kinematics.c` (131 行)
  - `lqr_controller.c` (154 行)
  - `chassis_control.c` (104 行)
  - `sensor_fusion.c` (90 行)

### PR #18 — 综合优化 (P4, P5, P9, P10)
- P5: `arm_sqrt_f32()` 后添加除零保护 `if(S < 1e-6) S = 1e-6`
- P4: 传感器融合中提取共享 IMU 变量避免重复读取
- P9: PID 参数改用语义化 `PID_PARAM(Kp,Ki,Kd,Alpha,Db,Li,Lo)` 宏
- P10: `control_io.h` 添加 `_Static_assert` 编译期大小检查
- `.gitignore` 添加 Keil 编译产物规则

详见 [`MDK-ARM/architecture_analysis_report.md`](MDK-ARM/architecture_analysis_report.md)

---

## 🔧 调试与可视化

### VOFA+ 遥测

`Control_Task` 主循环末尾通过 UART 向 VOFA+ 发送 4 通道数据：

```
USART_Vofa_Justfloat_Transmit(
    Target_Velocity,                    // 目标速度
    R_Leg_Info.Measure.Chassis_Velocity, // 右腿底盘速度
    L_Leg_Info.Measure.Chassis_Velocity, // 左腿底盘速度
    L_Leg_Info.Measure.Chassis_Position  // 底盘位置
);
```

在 VOFA+ 中选择 JustFloat 协议即可实时查看波形。

### SystemView 系统分析

项目集成了 SEGGER SystemView，可以分析 FreeRTOS 各任务的时间轴和 CPU 负载。

---

## 🤝 贡献指南

### Git 分支模型

```
main                           ← 稳定主线 (可编译, 可烧录)
├── feature/xxx                 ← 新功能/重构
├── bugfix/xxx                  ← Bug 修复
├── chore/xxx                   ← 文档/清理
└── experiment/xxx              ← 实验性改动
```

### 提交流程

1. `git checkout main && git pull`
2. `git checkout -b feature/your-change`
3. 写代码 → 小步提交 (`feat:`, `fix:`, `refactor:` 格式)
4. `git push -u origin feature/your-change`
5. 在 GitHub 上创建 Pull Request
6. Code Review 通过后合并

### 提交信息规范

```
feat:    新功能       → "feat: add PID auto-tuning"
fix:     修 bug       → "fix: add zero-guard for arm_sqrt_f32"
refactor: 重构        → "refactor: split Control_Task into 5 modules"
docs:    文档         → "docs: update README with architecture overview"
chore:   杂务         → "chore: add Keil artifacts to .gitignore"
```

---

## 📚 相关文档

| 文档 | 说明 |
|------|------|
| [`architecture_analysis_report.md`](MDK-ARM/architecture_analysis_report.md) | 完整代码审查报告 (核心架构、数据流、10条优化建议) |
| [`architecture_refactor_plan.md`](architecture_refactor_plan.md) | 重构方案与任务分解 |
| [`COD_H7_Template.ioc`](COD_H7_Template.ioc) | STM32CubeMX 引脚配置 (双击打开) |
| [GitHub Issues](https://github.com/Tensor-0/rm-biased-parallel-controller/issues) | 待办任务与 Bug 追踪 |
| [GitHub Pull Requests](https://github.com/Tensor-0/rm-biased-parallel-controller/pulls) | 合并历史 |

---

## ⚠ 注意事项

1. **首次烧录**：确保所有关节电机已安装并连接正确，否则机身可能突然摆动
2. **遥控器档位**：位置 1=高腿长, 2=关机, 3=初始化+平衡
3. **电池电压**：低于 22V 时请及时充电，避免异常复位
4. **编译环境**：修改 `.c` 文件后需在 Keil/EIDE 中重新添加新文件到工程
5. **电机 ID**：FDCAN2 挂载 4 个 DM8009 (ID 0-3)，FDCAN3 挂载 2 个 M3508 (ID 0x201-0x202)

---

## 📄 License

MIT License — 详见 [LICENSE](LICENSE) 文件
