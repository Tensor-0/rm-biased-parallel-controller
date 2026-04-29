/**
  ******************************************************************************
  * @file           : control_io.c
  * @brief          : 控制任务 I/O 边界实现
  * @description    : 输入快照采集和输出命令包打包。
  *                   这是整个控制层中唯一允许直接访问硬件全局变量的地方。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "control_io.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "bsp_adc.h"
#include "cmsis_os.h"

/* 全局输出命令包 —— CAN_Task 通过 extern 读取 */
motor_command_packet_t g_motor_cmd = {0};

/**
 * @brief  从全局硬件变量一次性采集输入快照
 * @note   此函数是控制层访问底层全局变量的唯一入口
 */
void Control_InputSnapshot_Update(control_input_snapshot_t *in)
{
    /* 时间戳 */
    in->tick = osKernelSysTick();

    /* IMU 数据 */
    in->ins = INS_Info;

    /* 遥控器 */
    in->rc.ch[0] = remote_ctrl.rc.ch[0];
    in->rc.ch[1] = remote_ctrl.rc.ch[1];
    in->rc.ch[2] = remote_ctrl.rc.ch[2];
    in->rc.ch[3] = remote_ctrl.rc.ch[3];
    in->rc.ch[4] = remote_ctrl.rc.ch[4];
    in->rc.s[0]  = remote_ctrl.rc.s[0];
    in->rc.s[1]  = remote_ctrl.rc.s[1];

    /* 4个关节电机 (DM8009) */
    for (int i = 0; i < 4; i++) {
        in->joint[i].position = DM_8009_Motor[i].Data.Position;
        in->joint[i].velocity = DM_8009_Motor[i].Data.Velocity;
        in->joint[i].torque   = DM_8009_Motor[i].Data.Torque;
        in->joint[i].state    = DM_8009_Motor[i].Data.State;
    }

    /* 2个驱动轮电机 */
    in->wheel[0].velocity = Chassis_Motor[0].Data.Velocity;
    in->wheel[1].velocity = Chassis_Motor[1].Data.Velocity;

    /* 电池电压 */
    in->vbat = USER_ADC_Voltage_Update();
}

/**
 * @brief  从控制状态生成输出命令包
 * @note   将 Control_Info 中的 SendValue 收敛为独立命令包
 */
void Control_OutputPacket_Generate(const Control_Info_Typedef *ctrl,
                                   motor_command_packet_t *out)
{
    /* 关节力矩: [0]=左小腿, [1]=左大腿, [2]=右大腿, [3]=右小腿 */
    out->joint_torque[0] = ctrl->L_Leg_Info.SendValue.T_Calf;
    out->joint_torque[1] = ctrl->L_Leg_Info.SendValue.T_Thigh;
    out->joint_torque[2] = ctrl->R_Leg_Info.SendValue.T_Thigh;
    out->joint_torque[3] = ctrl->R_Leg_Info.SendValue.T_Calf;

    /* 驱动轮电流 */
    out->wheel_current[0] = ctrl->L_Leg_Info.SendValue.Current;
    out->wheel_current[1] = ctrl->R_Leg_Info.SendValue.Current;

    /* 状态标志 */
    out->chassis_situation = (uint8_t)ctrl->Chassis_Situation;
    out->joint_init_done   = ctrl->Init.Joint_Init.IF_Joint_Init;

    /* 电机激活标志: 遥控器开关 s[1]==3(初始化) 或 s[1]==1(高腿长) 时为 1，
     * s[1]==2 或 0 时为 0。CAN_Task 用此值替代直接读取 remote_ctrl。*/
    out->motor_active = (uint8_t)(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 1);

    /* 序号和时间戳 */
    out->seq++;
    out->tick = osKernelSysTick();
}
