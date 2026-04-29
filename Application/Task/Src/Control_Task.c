/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : 控制任务编排器 (1kHz)
  * @description    : 将控制流水线委托给 5 个子模块:
  *                   mode_state_machine → vmc_kinematics → lqr_controller
  *                   → sensor_fusion → chassis_control
  ******************************************************************************
  */

#include "Control_Task.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Image_Transmission.h"
#include "control_io.h"

#include "mode_state_machine.h"
#include "vmc_kinematics.h"
#include "lqr_controller.h"
#include "chassis_control.h"
#include "sensor_fusion.h"

/* ---- 全局控制状态 ---- */
Control_Info_Typedef Control_Info = {
    .Base_Leg_Length_Low  = 0.14f,
    .Base_Leg_Length_High = 0.20f,
    .Roll = {
        .Distance_Two_Wheel = 0.4157f,
        .Offset = 0.0f,
        .Target_Angle = 0.0f,
    },
    .L_Leg_Info = {
        .Biased = {
            .L_Thigh_Link = 0.118114f,
            .L_Calf_Link  = 0.100f,
            .K = 0.465116f,
        },
        .Gravity_Compensation = 100.f,
        .Link_Gravity_Compensation_Angle = 0.0f,
        .Phi_Comp_Angle = -0.f,
    },
    .R_Leg_Info = {
        .Biased = {
            .L_Thigh_Link = 0.118114f,
            .L_Calf_Link  = 0.100f,
            .K = 0.465116f,
        },
        .Gravity_Compensation = 100.f,
        .Link_Gravity_Compensation_Angle = 0.4701917f,
        .Phi_Comp_Angle = -0.f,
    },
};

/* 输入快照（每周期采集一次） */
static control_input_snapshot_t g_ctrl_input;
TickType_t Control_Task_SysTick = 0;

void Control_Task(void const * argument)
{
    Mode_Init(&Control_Info);

    for (;;) {
        Control_Task_SysTick = osKernelSysTick();

        /* ===== 输入边界 ===== */
        Control_InputSnapshot_Update(&g_ctrl_input);

        /* ===== 1. 状态机与模式 ===== */
        Mode_Check_Low_Voltage(&Control_Info, &g_ctrl_input);
        Mode_Update(&Control_Info, &g_ctrl_input);

        /* ===== 2. VMC 运动学 ===== */
        VMC_Joint_Angle_Offset(&Control_Info, &g_ctrl_input);
        VMC_Calculate(&Control_Info);

        /* ===== 3. LQR 控制器 ===== */
        LQR_K_Update(&Control_Info);

        /* ===== 4. 传感器融合 ===== */
        SensorFusion_Measure_Update(&Control_Info, &g_ctrl_input);

        /* ===== 5. 底盘高层控制 ===== */
        Chassis_Move_Control(&Control_Info, &g_ctrl_input);
        Chassis_Height_Control(&Control_Info, &g_ctrl_input);
        Chassis_Roll_Control(&Control_Info, &g_ctrl_input);
        Leg_Length_Control(&Control_Info);

        /* ===== 6. VMC 力/力矩计算 ===== */
        VMC_Measure_F_Tp_Calculate(&Control_Info);

        /* ===== 7. LQR 输出 ===== */
        LQR_X_Update(&Control_Info);
        LQR_T_Tp_Calculate(&Control_Info);

        /* ===== 8. 关节力矩输出 ===== */
        VMC_Joint_Tourgue_Calculate(&Control_Info);

        /* ===== 输出边界 ===== */
        Control_OutputPacket_Generate(&Control_Info, &g_motor_cmd);

        /* ===== 调试遥测 ===== */
        USART_Vofa_Justfloat_Transmit(Control_Info.Target_Velocity,
                                       Control_Info.R_Leg_Info.Measure.Chassis_Velocity,
                                       Control_Info.L_Leg_Info.Measure.Chassis_Velocity,
                                       Control_Info.L_Leg_Info.Measure.Chassis_Position);

        osDelayUntil(&Control_Task_SysTick, 1);
    }
}
