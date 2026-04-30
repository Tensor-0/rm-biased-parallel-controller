/**
  ******************************************************************************
  * @file           : control_io.h
  * @brief          : 控制任务 I/O 边界 — 算法与硬件之间的"海关"
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   想象一个餐厅：后厨(硬件)不断产生食材(传感器数据)，前台(算法)需要
  *   做菜(计算控制量)。这个文件就是"菜单"和"出餐口"——它规定：
  *   1. 传感器数据必须按"菜单格式"(input_snapshot)打包好才能给算法
  *   2. 算法算完的命令必须按"出餐格式"(motor_cmd)打包好才能给硬件
  *
  *   为什么要有这个"海关"？因为算法层如果直接伸手去后厨拿数据,
  *   会出现"厨师正在切菜你去抢刀"的并发冲突——俗称数据竞争(Data Race)。
  *
  * 【核心职责】
  *   1. 定义 input_snapshot  —— 每 1ms 给所有传感器"拍照"的格式
  *   2. 定义 motor_cmd_packet —— 每 1ms 给电机下发命令的格式
  *   3. 编译期大小检查 (_Static_assert) —— 防止快照太大撑爆栈
  *
  * 【前置知识】
  *   - 结构体(struct): C语言的"数据包裹",把相关变量打包在一起
  *   - 快照(Snapshot): 一瞬间拷贝所有传感器状态,之后只看这个拷贝
  *   - CAN 总线: 机器人电机通信用的"网络电缆"
  *
  ******************************************************************************
  */

#ifndef CONTROL_IO_H
#define CONTROL_IO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "INS_Task.h"
#include "Control_Task.h"

/* ======================== 输入快照 (从硬件 → 算法) ======================== */

/**
 * @brief 关节电机反馈快照
 *
 * 🐣 通俗理解: 每条腿有2个关节(大腿+小腿),每个关节有一个DM8009电机。
 *            电机每时每刻都在汇报"我现在转了多少度,转得多快,用了多大力"。
 *            这个结构体就是这一瞬间的"体检数据"。
 */
typedef struct {
    /* [关节角度/位置] (弧度 rad) | 范围: [-π, π] | 电机转轴的实际位置,
       正方向由机械安装决定,左小腿以"腿伸直"为 0 附近 */
    float position;

    /* [关节角速度] (弧度/秒 rad/s) | 范围: [-30, 30] | 转动快慢,
       正值=逆时针转,负值=顺时针转 */
    float velocity;

    /* [关节输出力矩] (牛·米 N·m) | 范围: [-15, 15] | 电机实际输出的扭矩,
       用于反算腿承受的力 (逆动力学) */
    float torque;

    /* [电机状态字] | 0=未使能, 1=已使能 | 类似"车门锁没锁" */
    uint8_t state;
} joint_snapshot_t;

/**
 * @brief 驱动轮电机反馈快照
 *
 * 🐣 通俗理解: 两条腿底部的轮子各有一个M3508电机,负责让机器人前进后退。
 *            这里只取转速,因为轮子的控制是"电流模式"而非"位置模式"。
 */
typedef struct {
    /* [轮速] (转/分钟 RPM) | 范围: [-8000, 8000] | 正值=向前转 */
    int16_t velocity;
} wheel_snapshot_t;

/**
 * @brief 遥控器快照
 *
 * 🐣 通俗理解: 遥控器上每根摇杆和开关都有一个"通道号"。
 *            ch[0]~ch[4]=5根摇杆的位置, s[0]~s[1]=2个开关的位置。
 */
typedef struct {
    int16_t ch[5];    /* 通道值 | 摇杆范围: [-660, 660] */
    uint8_t s[2];     /* 开关值 | 0/1/2/3,对应不同档位 */
} rc_snapshot_t;

/**
 * @brief 控制任务输入快照 — 整个算法的"视网膜"
 *
 * 🐣 这是全系统最重要的结构体之一！每 1ms 执行一次"拍照"：
 *   - 把所有传感器(IMU/遥控器/4个关节电机/2个轮子电机)的数据
 *     一秒不差地拷贝到这里
 *   - 之后 16 步控制流水线只看这个快照,不再问硬件
 *   - 这保证了"在同一时刻看到的世界是一致的"
 *
 * ⚠ 为什么大小不能超过 256 字节？
 *   因为 1ms 一次、每次都在栈上分配。太大会栈溢出导致 HardFault。
 */
typedef struct {
    INS_Info_Typedef ins;       /* IMU 姿态: 欧拉角[3]+角速度[3]+加速度[3] */
    rc_snapshot_t rc;           /* 遥控器: 摇杆+开关 */
    joint_snapshot_t joint[4];  /* 4个关节电机: [0]左小腿 [1]左大腿 [2]右大腿 [3]右小腿 */
    wheel_snapshot_t wheel[2];  /* 2个驱动轮: [0]左轮 [1]右轮 */
    float vbat;                 /* [电池电压] (伏特 V) | 正常: [22, 25.2] | <22V需报警 */
    uint32_t tick;              /* 采样时间戳 (FreeRTOS tick) */
} control_input_snapshot_t;

/* ======================== 输出命令包 (从算法 → 硬件) ======================== */

/**
 * @brief 电机命令包 — 算法算完后下发给电机的"圣旨"
 *
 * 🐣 通俗理解: Control_Task 算了 16 步之后,得到了"4个关节各该用多大力,
 *            2个轮子各该用多大电流"。把这个结果打包成一个"命令包",
 *            CAN_Task 只负责把这个包发出去——它不管这个值是怎么算出来的。
 *
 * 📦 关节顺序 (一定要记住！):
 *    [0]=左小腿 [1]=左大腿 [2]=右大腿 [3]=右小腿
 */
typedef struct {
    /* [关节目标力矩] (N·m) | 范围: [-15, 15] | 正值=伸腿,负值=收腿
       ⚠ 如果此值超限, Joint_Tourgue_Calculate 中已有限幅保护 */
    float joint_torque[4];

    /* [驱动轮目标电流] (mA) | 范围: [-10000, 10000] | [0]=左轮 [1]=右轮
       🐣 注意右轮取反了: 因为左右轮镜像安装,同时正转应该是反方向 */
    int16_t wheel_current[2];

    /* [底盘状态] 0=CHASSIS_WEAK(虚弱,关机), 1=CHASSIS_BALANCE(平衡,运行) */
    uint8_t chassis_situation;

    /* [关节初始化完成标志] 0=初始化中 1=已完成 | 初始化期间电机只做归零动作 */
    uint8_t joint_init_done;

    /* [电机激活标志] 0=全部断电, 1=正常运行
       🐣 这个标志由 Control_Task 根据遥控器开关 s[1] 的值来设置:
       s[1]==3(初始化) 或 s[1]==1(高腿长) → 激活=1
       s[1]==2(关机)   或 s[1]==0         → 激活=0 */
    uint8_t motor_active;

    uint8_t reserved;           /* 预留对齐 (凑 4 字节对齐) */

    uint32_t seq;               /* [命令序号] 每次打包+1, 用于检测丢包 */
    uint32_t tick;              /* [打包时间戳] FreeRTOS tick */
} motor_command_packet_t;

/* 🔧 编译期安全检查:
   如果这个断言失败,说明 control_input_snapshot_t 超过了 256 字节,
   在 1kHz 频率下可能导致栈溢出。需要检查是否添加了太多不必要的字段。 */
_Static_assert(sizeof(control_input_snapshot_t) <= 256,
               "control_input_snapshot_t too large for 1kHz stack allocation");

/* ======================== 函数声明 ======================== */

/**
 * @brief  "拍照" — 采集所有传感器的当前值
 *
 * 🐣 这是整个系统中唯一可以直接访问硬件全局变量的地方。
 *    (DM_8009_Motor, Chassis_Motor, remote_ctrl, INS_Info 等)
 *    其他所有代码都通过这个快照间接读取,从而避免数据竞争。
 */
void Control_InputSnapshot_Update(control_input_snapshot_t *in);

/**
 * @brief  "打包" — 把算好的控制量封装成电机命令包
 *
 * 🐣 只做赋值操作: 把 Control_Info 中的 SendValue 字段拷贝到 g_motor_cmd。
 *    不做任何计算。CAN_Task 后续只读 g_motor_cmd。
 */
void Control_OutputPacket_Generate(const Control_Info_Typedef *ctrl,
                                   motor_command_packet_t *out);

/* ======================== 全局对象 ======================== */

/** g_motor_cmd — 唯一的电机命令出口
 *  Control_Task 写入, CAN_Task 只读, 不需要锁 */
extern motor_command_packet_t g_motor_cmd;

#endif /* CONTROL_IO_H */
