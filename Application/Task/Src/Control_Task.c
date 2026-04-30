/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : 控制任务编排器 (1kHz) — 机器人的"大脑皮层"
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   这是整个项目最核心的文件！它是机器人的"总司令"，每 1 毫秒执行一次
  *   "思考循环"：看一眼世界(快照) → 做一连串计算 → 下达命令(电机指令)。
  *   但这个文件本身**不做任何计算**——它只是"叫手下干活"的编排器(Orchestrator)。
  *
  * 【核心职责】
  *   1. 初始化 PID 控制器参数 (一次性)
  *   2. 每 1ms 循环一次,按固定顺序调度 11 步控制流水线
  *   3. 流水线开始前采集快照,结束后打包命令
  *   4. 每周期发送 4 通道调试数据到 VOFA+ 上位机
  *
  * 【运行频率】 1kHz = 每 1 毫秒执行一次
  *   为什么这么快？因为机器人就像一根倒立的扫帚——平衡是"动态"的。
  *   如果你 10ms 才反应一次(100Hz),那扫帚早就倒了。
  *   1kHz = 人类感知不到延迟,对机器人来说已经够快。
  *
  * 【前置知识】
  *   - FreeRTOS 的 osDelayUntil: 不是"睡 1ms"而是"等够 1ms",保证频率精准
  *   - 控制周期: 传感器的采样→算法计算→电机指令下发,必须在 1ms 内完成
  *   - 编排器模式: 自己不做具体工作,只调度子模块按顺序执行
  *
  ******************************************************************************
  */

#include "Control_Task.h"
#include "cmsis_os.h"          /* FreeRTOS 操作系统接口 */
#include "bsp_uart.h"          /* 串口驱动,用于 VOFA+ 调试遥测 */
#include "Image_Transmission.h" /* 图传模块 */
#include "control_io.h"         /* I/O 边界: 快照采集 + 命令打包 */

#include "mode_state_machine.h" /* 子模块1: 状态机 */
#include "vmc_kinematics.h"     /* 子模块2: VMC 运动学 */
#include "lqr_controller.h"     /* 子模块3: LQR 控制器 */
#include "chassis_control.h"    /* 子模块4: 底盘控制 */
#include "sensor_fusion.h"      /* 子模块5: 传感器融合 */

/* ====== 全局控制状态 ====== */
/**
 * Control_Info — 整个控制系统的"数据中心"
 *
 * 🐣 通俗理解: 这是机器人的"记忆"。里面记录了两条腿的物理参数、
 *   当前状态、目标状态、PID 输出、LQR 输出……一切控制相关的数据
 *   都存在这个结构体里。所有子模块都通过指针操作它。
 *
 *   初始化值(机械参数): 这些是"说明书上的参数"——腿有多长、杆长比多少。
 *   如果是新车、新机械结构,这些值要改。
 */
Control_Info_Typedef Control_Info = {
    /* [基础低腿长] (米 m) | 范围: [0.10, 0.25] | 低底盘模式下的腿长
       🐣 腿长不是"骨头长度",而是从髋关节到脚底的虚拟长度 */
    .Base_Leg_Length_Low  = 0.14f,
    /* [基础高腿长] (米 m) | 范围: [0.15, 0.35] | 高底盘模式下的腿长 */
    .Base_Leg_Length_High = 0.20f,

    .Roll = {
        /* [两轮机械间距] (米 m) | 正常: 0.40~0.45 | 左右轮子中心之间的距离
           🐣 这个值直接影响横滚补偿量: 间距越大 → 腿长差产生的横滚角越小 */
        .Distance_Two_Wheel = 0.4157f,
        .Offset = 0.0f,           /* IMU 角度补偿 */
        .Target_Angle = 0.0f,     /* 目标横滚角=0 (保持水平) */
    },

    .L_Leg_Info = {
        .Biased = {
            /* [大腿连杆长度] (米 m) | AH段长度 | 动力长后杆 */
            .L_Thigh_Link = 0.118114f,
            /* [小腿连杆长度] (米 m) | AD段长度 | 动力短前杆 */
            .L_Calf_Link  = 0.100f,
            /* [连杆长度比] K = AD/AH < 1 | 比值越小,同角度变化腿长变化越大 */
            .K = 0.465116f,
        },
        /* [重力补偿] (牛顿 N) | 正常: 80~120 | 抵消机器人自重需要的推力
           🐣 这个值≈机器人总重/2,因为两条腿各扛一半 */
        .Gravity_Compensation = 100.f,
        /* [连杆重心补偿角度] (弧度 rad) | 用于修正连杆自身重量的影响 */
        .Link_Gravity_Compensation_Angle = 0.0f,
        /* [陀螺仪角度补偿] (弧度 rad) | IMU 安装偏差的补偿值
           🐣 当车平衡时陀螺仪读数非零,就用这个值补偿回去 */
        .Phi_Comp_Angle = -0.f,
    },

    .R_Leg_Info = {
        .Biased = {
            .L_Thigh_Link = 0.118114f,
            .L_Calf_Link  = 0.100f,
            .K = 0.465116f,
        },
        .Gravity_Compensation = 100.f,
        /* ⚠ 左右腿的补偿角度可能不同(机械不对称),所以这里写了 0.47 */
        .Link_Gravity_Compensation_Angle = 0.4701917f,
        .Phi_Comp_Angle = -0.f,
    },
};

/* [输入快照] 每周期采集一次,存在全局区(非栈)方便调试 */
static control_input_snapshot_t g_ctrl_input;

/* [控制周期时间戳] 用于 osDelayUntil 保证 1kHz 精确频率 */
TickType_t Control_Task_SysTick = 0;

/**
 * Control_Task — 控制周期主循环 (1kHz FreeRTOS 线程)
 *
 * 🐣 这个函数只做一件事: 按固定顺序调用 11 个子步骤,
 *    每个步骤都委托给专门的子模块。你可以把它理解为
 *    "机器人每秒做 1000 次下图的流程":
 *
 *  快照 ──▶ 状态机 ──▶ VMC运动学 ──▶ LQR增益 ──▶ 传感器融合
 *                              │
 *      底盘控制 ◀────────────────────┘
 *         │
 *      力/力矩计算 ──▶ LQR输出 ──▶ 关节力矩 ──▶ 打包命令 ──▶ 串口遥测
 *
 *  Step 0: 采集快照     — "拍照"当前世界状态
 *  Step 1: 低电压检测   — 电池还有电吗？
 *  Step 2: 模式更新     — 现在该平衡还是关机？
 *  Step 3: 关节映射     — 电机角度→腿的角度
 *  Step 4: VMC正运动学  — 复杂腿→简单棍子(L₀, φ₀)
 *  Step 5: LQR增益更新  — 根据腿长查表得到最优推力系数
 *  Step 6: 传感器融合   — IMU + 轮速计 → 6维状态向量
 *  Step 7~10: 底盘控制  — 前进/转向/高度/横滚/腿长
 *  Step 11: 力/力矩     — 算腿上的力和扭矩
 *  Step 12: LQR输出     — u = -K·X 算出该用多大力
 *  Step 13: 关节力矩    — 雅可比矩阵→4个电机的真实扭矩
 *  Step 14: 打包命令    — 格式化为 g_motor_cmd
 *  Step 15: 串口遥测    — 发调试数据到上位机
 */
void Control_Task(void const * argument)
{
    /* 🐣 第一步(只跑一次): 把所有 PID 控制器初始化好
       类似"开机自检"——设置 Kp/Ki/Kd 参数、清零积分项 */
    Mode_Init(&Control_Info);

    for (;;) {
        /* 🐣 osKernelSysTick() 返回"系统从开机到现在走了多少毫秒"
           用来配合 osDelayUntil 实现精确的 1ms 定时 */
        Control_Task_SysTick = osKernelSysTick();

        /* ====== 输入边界: 对所有传感器"拍照" ======
           🐣 这一瞬间拷贝了所有硬件状态,后续14步都不会再碰硬件。
           这就像是"闭上眼睛,凭记忆走路"——虽然记忆只有1ms前,但足够精确。 */
        Control_InputSnapshot_Update(&g_ctrl_input);

        /* ====== 第1步 低电压检测 ======
           🐣 只看一眼电池电压,不报警不做任何事。
           如果电压 < 22V 蜂鸣器会叫(已注释掉,简化实现) */
        Mode_Check_Low_Voltage(&Control_Info, &g_ctrl_input);

        /* ====== 第2步 模式更新 ======
           🐣 根据遥控器开关位置,决定机器人的状态:
           s[1]=3 → 初始化模式,关节归位
           s[1]=1 → 高腿长 + 平衡
           s[1]=2 → 关机,所有电机关闭
           初始化逻辑中 4 个关节必须同时到达安全位置才算"初始化完成" */
        Mode_Update(&Control_Info, &g_ctrl_input);

        /* ====== 第3步 关节角度映射 ======
           🐣 电机汇报的是"转轴角度",但算法需要的是"大腿/小腿摆角"。
           这里有坐标系转换: 左大腿 = PI + 关节1角度 (因为电机方向和腿方向差180°) */
        VMC_Joint_Angle_Offset(&Control_Info, &g_ctrl_input);

        /* ====== 第4步 VMC正运动学 ======
           🐣 核心步骤！把两条复杂的连杆腿变成两根简单的"虚拟棍子"。
           输入: 大腿角度 + 小腿角度
           输出: 虚拟腿长 L₀ + 虚拟腿倾角 φ₀
           数学本质: 连杆正运动学 → 几何简化 */
        VMC_Calculate(&Control_Info);

        /* ====== 第5步 LQR增益更新 ======
           🐣 腿长变了,K 矩阵就得跟着变。因为腿越长杠杆效应越大,需要的力越小。
           K(L₀) = 多项式(L₀³, L₀², L₀, 常数),用霍纳法则快速计算 */
        LQR_K_Update(&Control_Info);

        /* ====== 第6步 传感器融合 ======
           🐣 IMU 给你倾角和角速度,轮速计给你底盘速度。但两个都不完美。
           融合策略: 0.8×预测 + 0.2×测量 = 既快又稳的速度估计。
           同时计算加速度(用于速度预测),并检测 CHASSIS_WEAK 状态清零所有状态量 */
        SensorFusion_Measure_Update(&Control_Info, &g_ctrl_input);

        /* ====== 第7~10步 底盘高层控制 ======
           🐣 四个独立的控制通道,按顺序执行: */
        /* 7. 前进/后退/转向: 遥控器摇杆 → 速度目标 + 偏航力矩 */
        Chassis_Move_Control(&Control_Info, &g_ctrl_input);
        /* 8. 高度切换: 遥控器开关 → 基础腿长目标 */
        Chassis_Height_Control(&Control_Info, &g_ctrl_input);
        /* 9. 横滚补偿: IMU Roll角 → 左右腿长差 + 推力差 */
        Chassis_Roll_Control(&Control_Info, &g_ctrl_input);
        /* 10.腿长控制: 基础+补偿 → 总目标腿长 → PID 计算推力 */
        Leg_Length_Control(&Control_Info);

        /* ====== 第11步 VMC力/力矩计算 ======
           🐣 从关节电机反馈的力矩,反推虚拟腿受到的外力。
           同时计算地面支持力 FN,判断腿是否着地(Support.Flag)。
           为什么需要这个？空中的腿蹬不到地,再用力也是浪费。
           所以检测到离地后,LQR 会把那条腿的推力清零。 */
        VMC_Measure_F_Tp_Calculate(&Control_Info);

        /* ====== 第12步 LQR 状态误差 ======
           🐣 X = Target - Measure。算出差了多少。
           Target 一般都是0: "我希望不倾斜、不偏移、不移动"*/
        LQR_X_Update(&Control_Info);

        /* ====== 第13步 LQR 控制输出 ======
           🐣 u = -K·X: K 告诉"差这么多该用多大力",
           X 告诉"现在差多少"。两者相乘得到最优控制力。
           同时执行"防劈叉"PID(防止两条腿越走越散)和力矩综合。 */
        LQR_T_Tp_Calculate(&Control_Info);

        /* ====== 第14步 关节力矩映射 ======
           🐣 虚拟腿的推力 F 和扭矩 Tp → 4 个关节电机各该输出多少扭矩。
           通过 2×2 雅可比矩阵转换,同时限幅防止力矩超标烧电机。 */
        VMC_Joint_Tourgue_Calculate(&Control_Info);

        /* ====== 输出边界: 打包命令 ======
           🐣 把所有 SendValue 拷贝到 g_motor_cmd。
           CAN_Task 会另起线程,从这里读取并发给电机。 */
        Control_OutputPacket_Generate(&Control_Info, &g_motor_cmd);

        /* ====== 调试遥测 ======
           🐣 向 VOFA+ 串口上位机发送 4 个 float 数据(JustFloat 协议)。
           这 4 个通道可以在电脑上实时看波形:
           [目标速度 | 右腿底盘速度 | 左腿底盘速度 | 底盘位置]
           方便调试时观察控制效果 */
        USART_Vofa_Justfloat_Transmit(Control_Info.Target_Velocity,
                                       Control_Info.R_Leg_Info.Measure.Chassis_Velocity,
                                       Control_Info.L_Leg_Info.Measure.Chassis_Velocity,
                                       Control_Info.L_Leg_Info.Measure.Chassis_Position);

        /* 🐣 osDelayUntil 不是"睡 1ms",而是"等到离上次执行刚好 1ms 为止"。
           这样可以抵抗计算时间抖动,保证精确的 1kHz 频率。
           如果某次计算花了 0.9ms,那这次只会等 0.1ms。
           如果某次计算花了 1.1ms(超了!),那就立刻执行(不睡),频率会暂时下降。 */
        osDelayUntil(&Control_Task_SysTick, 1);
    }
}
