/**
  ******************************************************************************
  * @file           : control_io.h
  * @brief          : 控制任务 I/O 边界定义
  * @description    : 定义输入快照和输出命令包结构体，
  *                   将控制算法与硬件全局变量解耦。
  ******************************************************************************
  */

#ifndef CONTROL_IO_H
#define CONTROL_IO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "INS_Task.h"
#include "Control_Task.h"

/* ======================== 输入快照 ======================== */

/**
 * @brief 关节电机反馈快照 (对应 DM_8009_Motor[i].Data)
 */
typedef struct {
    float position;   /* 关节位置 (rad) */
    float velocity;   /* 关节角速度 (rad/s) */
    float torque;     /* 关节力矩反馈 (N·m) */
    uint8_t state;    /* 电机状态字 */
} joint_snapshot_t;

/**
 * @brief 驱动轮电机反馈快照 (对应 Chassis_Motor[i].Data)
 */
typedef struct {
    int16_t velocity;  /* 轮速 (RPM) */
} wheel_snapshot_t;

/**
 * @brief 遥控器快照 (对应 remote_ctrl.rc)
 */
typedef struct {
    int16_t ch[5];    /* 通道值 */
    uint8_t s[2];     /* 开关值 */
} rc_snapshot_t;

/**
 * @brief 控制任务输入快照
 * @note  每个控制周期开始时一次性采集，后续控制算法只读此结构体
 */
typedef struct {
    INS_Info_Typedef ins;       /* IMU 姿态/角速度/加速度 */
    rc_snapshot_t rc;           /* 遥控器通道和开关 */
    joint_snapshot_t joint[4];  /* 4个关节电机 (DM8009) */
    wheel_snapshot_t wheel[2];  /* 2个驱动轮电机 */
    float vbat;                 /* 电池电压 (V) */
    uint32_t tick;              /* 采样时间戳 */
} control_input_snapshot_t;

/* ======================== 输出命令包 ======================== */

/**
 * @brief 电机命令包
 * @note  Control_Task 每周期结束时打包，CAN_Task 只读此结构体
 *        关节顺序: [0]=左小腿, [1]=左大腿, [2]=右大腿, [3]=右小腿
 */
typedef struct {
    float joint_torque[4];      /* 关节目标力矩 (N·m) */
    int16_t wheel_current[2];   /* 驱动轮目标电流: [0]=左, [1]=右 */
    uint8_t chassis_situation;  /* 底盘状态 (CHASSIS_WEAK/CHASSIS_BALANCE) */
    uint8_t joint_init_done;    /* 关节初始化完成标志 */
    uint8_t motor_active;       /* 电机激活标志: 0=关机, 1=运行 (由 Control_Task 根据遥控器开关填充) */
    uint8_t reserved;           /* 预留对齐 */
    uint32_t seq;               /* 命令序号 */
    uint32_t tick;              /* 打包时间戳 */
} motor_command_packet_t;

/* ======================== 函数声明 ======================== */

/**
 * @brief  从全局硬件变量一次性采集输入快照
 * @param  in: 输出的快照结构体指针
 */
void Control_InputSnapshot_Update(control_input_snapshot_t *in);

/**
 * @brief  从控制状态生成输出命令包
 * @param  ctrl: 控制状态结构体指针
 * @param  out:  输出的命令包指针
 */
void Control_OutputPacket_Generate(const Control_Info_Typedef *ctrl,
                                   motor_command_packet_t *out);

/* ======================== 全局对象声明 ======================== */

extern motor_command_packet_t g_motor_cmd;

#endif /* CONTROL_IO_H */
