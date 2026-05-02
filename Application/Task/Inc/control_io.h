/**
  ******************************************************************************
  * @file           : control_io.h
  * @brief          : 控制任务 I/O 边界 — 算法与硬件之间的"海关"
  *
  *  改进版 (v3.1):
  *   - 新增 FreeRTOS 队列支持 (xQueueOverwrite/xQueuePeek)
  *   - g_motor_cmd 队列版替代原始共享变量，消除数据竞争
  *   - 切换方式: 取消注释 #define USE_QUEUE 即启用队列
  ******************************************************************************
  */

#ifndef CONTROL_IO_H
#define CONTROL_IO_H

#include "stdint.h"
#include "stdbool.h"
#include "INS_Task.h"
#include "Control_Task.h"
#include "Robot_Config.h"

/* ====== 队列模式开关 ======
    取消下面这行的注释即可启用 FreeRTOS 队列,彻底消除数据竞争。
   启用后需要: FreeRTOS 初始化时调用 xQueueCreate() 创建队列。
   未启用时使用原始共享变量(兼容当前版本)。 */
/* #define USE_QUEUE */

#ifdef USE_QUEUE
#include "cmsis_os.h"
#endif

/* ======================== 输入快照 ======================== */

typedef struct {
    float position;    /* [关节角度] (rad) | [-π, π] */
    float velocity;    /* [关节角速度] (rad/s) | [-30, 30] */
    float torque;      /* [关节力矩反馈] (N·m) | [-15, 15] */
    uint8_t state;     /* [电机状态字] | 0=未使能, 1=已使能 */
} joint_snapshot_t;

typedef struct {
    int16_t velocity;  /* [轮速] (RPM) | [-8000, 8000] */
} wheel_snapshot_t;

typedef struct {
    int16_t ch[5];     /* [通道值] | 摇杆[-660,660] */
    uint8_t s[2];      /* [开关值] | 0/1/2/3 */
} rc_snapshot_t;

typedef struct {
    INS_Info_Typedef ins;
    rc_snapshot_t rc;
    joint_snapshot_t joint[4];
    wheel_snapshot_t wheel[2];
    float vbat;        /* [电池电压] (V) | 正常: [22, 25.2] */
    uint32_t tick;     /* [时间戳] FreeRTOS tick */
} control_input_snapshot_t;

/* ======================== 输出命令包 ======================== */

typedef struct {
    float joint_torque[4];    /* [关节力矩] (N·m) | [0]=左小腿 [1]=左大腿 [2]=右大腿 [3]=右小腿 */
    int16_t wheel_current[2]; /* [轮子电流] (mA) | [0]=左 [1]=右 */
    uint8_t chassis_situation; /* [底盘状态] 0=WEAK 1=BALANCE */
    uint8_t joint_init_done;   /* [关节初始化完成标志] */
    uint8_t motor_active;      /* [电机激活标志] 0=关机 1=运行 */
    uint8_t reserved;          /* 预留对齐 */
    uint32_t seq;              /* [命令序号] */
    uint32_t tick;             /* [打包时间戳] */
} motor_command_packet_t;

_Static_assert(sizeof(control_input_snapshot_t) <= 256,
               "control_input_snapshot_t too large for 1kHz stack allocation");

/* ======================== 函数声明 ======================== */

void Control_InputSnapshot_Update(control_input_snapshot_t *in);
void Control_OutputPacket_Generate(const Control_Info_Typedef *ctrl,
                                   motor_command_packet_t *out);

/* ======================== 全局对象 ======================== */

#ifdef USE_QUEUE
/*  改进版: 使用 FreeRTOS 队列传递命令,完全消除数据竞争
   Control_Task 用 xQueueOverwrite(cmd_queue, &g_motor_cmd) 写入,
   CAN_Task 用 xQueuePeek(cmd_queue, &local_cmd, 0) 读取。
   创建队列: main() 中调用 cmd_queue = xQueueCreate(1, sizeof(motor_command_packet_t)); */
extern QueueHandle_t motor_cmd_queue;
#else
/* 当前版本: 共享全局变量,频率一致的 RTOS 任务间基本安全 */
extern motor_command_packet_t g_motor_cmd;
#endif

#endif /* CONTROL_IO_H */
