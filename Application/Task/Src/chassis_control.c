/**
  ******************************************************************************
  * @file           : chassis_control.c
  * @brief          : High-level chassis motion control
  ******************************************************************************
  */

#include "chassis_control.h"
#include "PID.h"
#include "Ramp.h"
#include "arm_math.h"

/* PID 引用 (定义在 mode_state_machine.c) */
extern PID_Info_TypeDef PID_Leg_length_F[2];
extern PID_Info_TypeDef PID_Leg_Roll_F;
extern PID_Info_TypeDef PID_Yaw[2];

void Chassis_Move_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    float K_Velocity = 0.001f;
    float K_Brake    = 0.002f;
    float K_Yaw_P    = 0.2f;

    /* 前进/后退 */
    if (in->rc.ch[3] != 0) {
        Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity, -in->rc.ch[3] * 0.00242f, K_Velocity);
        Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
    } else {
        Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity, 0, K_Brake);
    }
    VAL_LIMIT(Control_Info->Target_Velocity, -1.6f, 1.6f);

    /* 偏航转向 */
    Control_Info->Yaw_Err = f_Ramp_Calc(Control_Info->Yaw_Err, (in->rc.ch[2] * RemoteToDegrees), K_Yaw_P);
    if (Control_Info->Yaw_Err >= 180.f)
        Control_Info->Yaw_Err -= 360.f;
    else if (Control_Info->Yaw_Err <= -180.f)
        Control_Info->Yaw_Err += 360.f;

    PID_Calculate(&PID_Yaw[0], 0, Control_Info->Yaw_Err);
    PID_Calculate(&PID_Yaw[1], PID_Yaw[0].Output, in->ins.Yaw_Gyro);
    Control_Info->L_Leg_Info.Moment.Turn_T =  PID_Yaw[1].Output;
    Control_Info->R_Leg_Info.Moment.Turn_T = -PID_Yaw[1].Output;
}

void Chassis_Height_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    float K_Height = 0.00030f;
    if (in->rc.s[1] == 1) {
        Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_High, K_Height);
        Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_High, K_Height);
    } else {
        Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_Low, K_Height);
        Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_Low, K_Height);
    }
}

void Chassis_Roll_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    Control_Info->Roll.Length_Diff = Control_Info->L_Leg_Info.Sip_Leg_Length - Control_Info->R_Leg_Info.Sip_Leg_Length;
    Control_Info->Roll.Angle       = f_Ramp_Calc(Control_Info->Roll.Angle, (-in->ins.Angle[1] - 0.0291f), 0.003f);
    Control_Info->Roll.Tan_Length_Diff_Angle = Control_Info->Roll.Length_Diff / Control_Info->Roll.Distance_Two_Wheel;
    Control_Info->Roll.Tan_Angle   = arm_sin_f32(Control_Info->Roll.Angle) / arm_cos_f32(Control_Info->Roll.Angle);
    Control_Info->Roll.Tan_Slope_Angle = (Control_Info->Roll.Tan_Angle + Control_Info->Roll.Tan_Length_Diff_Angle)
                                       / (1.0f - (Control_Info->Roll.Tan_Angle * Control_Info->Roll.Tan_Length_Diff_Angle));

    Control_Info->L_Leg_Info.Roll_Leg_Length =  (Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel) / 2.0f;
    Control_Info->R_Leg_Info.Roll_Leg_Length = -(Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel) / 2.0f;

    Control_Info->L_Leg_Info.Moment.Roll_F = -PID_Calculate(&PID_Leg_Roll_F, 0.f, Control_Info->Roll.Angle);
    Control_Info->R_Leg_Info.Moment.Roll_F =  PID_Calculate(&PID_Leg_Roll_F, 0.f, Control_Info->Roll.Angle);
}

void Leg_Length_Control(Control_Info_Typedef *Control_Info)
{
    float K_Length = 0.35f;

    Control_Info->L_Leg_Info.Total_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Total_Leg_Length,
        Control_Info->L_Leg_Info.Base_Leg_Length + Control_Info->L_Leg_Info.Roll_Leg_Length, K_Length);
    Control_Info->R_Leg_Info.Total_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Total_Leg_Length,
        Control_Info->R_Leg_Info.Base_Leg_Length + Control_Info->R_Leg_Info.Roll_Leg_Length, K_Length);

    VAL_LIMIT(Control_Info->L_Leg_Info.Total_Leg_Length, 0.14f, 0.32f);
    VAL_LIMIT(Control_Info->R_Leg_Info.Total_Leg_Length, 0.14f, 0.32f);

    Control_Info->L_Leg_Info.Moment.Leg_Length_F = PID_Calculate(&PID_Leg_length_F[0], Control_Info->L_Leg_Info.Total_Leg_Length, Control_Info->L_Leg_Info.Sip_Leg_Length);
    Control_Info->R_Leg_Info.Moment.Leg_Length_F = PID_Calculate(&PID_Leg_length_F[1], Control_Info->R_Leg_Info.Total_Leg_Length, Control_Info->R_Leg_Info.Sip_Leg_Length);

    /* 重力补偿 + 支撑腿回退 */
    Control_Info->R_Leg_Info.Gravity_Compensation = 100.f;
    Control_Info->L_Leg_Info.Gravity_Compensation = 100.f;
    if (Control_Info->L_Leg_Info.Support.Flag == 1) {
        Control_Info->L_Leg_Info.Moment.Roll_F = 0;
        Control_Info->L_Leg_Info.Gravity_Compensation = 100.f;
    }
    if (Control_Info->R_Leg_Info.Support.Flag == 1) {
        Control_Info->R_Leg_Info.Moment.Roll_F = 0;
        Control_Info->R_Leg_Info.Gravity_Compensation = 100.f;
    }

    Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F + Control_Info->L_Leg_Info.Moment.Roll_F + Control_Info->L_Leg_Info.Gravity_Compensation;
    Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F + Control_Info->R_Leg_Info.Moment.Roll_F + Control_Info->R_Leg_Info.Gravity_Compensation;
}
