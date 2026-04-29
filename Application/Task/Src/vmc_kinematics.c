/**
  ******************************************************************************
  * @file           : vmc_kinematics.c
  * @brief          : VMC forward kinematics and torque mapping
  ******************************************************************************
  */

#include "vmc_kinematics.h"
#include "arm_math.h"

void VMC_Joint_Angle_Offset(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* 左腿 */
    Control_Info->L_Leg_Info.Biased.Calf_Angle      = in->joint[0].position;
    Control_Info->L_Leg_Info.Biased.Thigh_Angle     = PI + in->joint[1].position;
    Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot = in->joint[1].velocity;
    Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot  = in->joint[0].velocity;
    Control_Info->L_Leg_Info.Biased.T_Thigh         = in->joint[1].torque;
    Control_Info->L_Leg_Info.Biased.T_Calf          = in->joint[0].torque;

    /* 右腿 */
    Control_Info->R_Leg_Info.Biased.Thigh_Angle     = in->joint[2].position;
    Control_Info->R_Leg_Info.Biased.Calf_Angle      = PI + in->joint[3].position;
    Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot = in->joint[2].velocity;
    Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot  = in->joint[3].velocity;
    Control_Info->R_Leg_Info.Biased.T_Thigh         = in->joint[2].torque;
    Control_Info->R_Leg_Info.Biased.T_Calf          = in->joint[3].torque;
}

void VMC_Calculate(Control_Info_Typedef *Control_Info)
{
    /* 左腿 VMC 正运动学 — 中间变量使用栈局部变量 */
    float L_a = Control_Info->L_Leg_Info.Biased.L_Calf_Link;
    float L_b = Control_Info->L_Leg_Info.Biased.L_Thigh_Link;
    float L_M = -(Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle) / 2.f;
    float L_N = (Control_Info->L_Leg_Info.Biased.Calf_Angle + Control_Info->L_Leg_Info.Biased.Thigh_Angle) / 2.f;
    float L_S_Radicand = L_b * L_b - L_a * L_a * arm_sin_f32(L_M) * arm_sin_f32(L_M);
    float L_S;
    arm_sqrt_f32(L_S_Radicand, &L_S);
    float L_t = L_a * arm_cos_f32(L_M) + L_S;
    float L_A = (L_a * L_t * arm_sin_f32(L_M)) / L_S;

    Control_Info->L_Leg_Info.A = L_A;
    Control_Info->L_Leg_Info.Sip_Leg_Angle  = L_N;
    Control_Info->L_Leg_Info.Sip_Leg_Length = L_t / Control_Info->L_Leg_Info.Biased.K;

    Control_Info->L_Leg_Info.X_J_Dot = (L_A * arm_cos_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
                                      - L_t * arm_sin_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->L_Leg_Info.Biased.K);
    Control_Info->L_Leg_Info.Y_J_Dot = (L_A * arm_sin_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
                                      + L_t * arm_cos_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->L_Leg_Info.Biased.K);

    /* 右腿 VMC 正运动学 */
    float R_a = Control_Info->R_Leg_Info.Biased.L_Calf_Link;
    float R_b = Control_Info->R_Leg_Info.Biased.L_Thigh_Link;
    float R_M = -(Control_Info->R_Leg_Info.Biased.Thigh_Angle - Control_Info->R_Leg_Info.Biased.Calf_Angle) / 2.f;
    float R_N = (Control_Info->R_Leg_Info.Biased.Calf_Angle + Control_Info->R_Leg_Info.Biased.Thigh_Angle) / 2.f;
    float R_S_Radicand = R_b * R_b - R_a * R_a * arm_sin_f32(R_M) * arm_sin_f32(R_M);
    float R_S;
    arm_sqrt_f32(R_S_Radicand, &R_S);
    float R_t = R_a * arm_cos_f32(R_M) + R_S;
    float R_A = (R_a * R_t * arm_sin_f32(R_M)) / R_S;

    Control_Info->R_Leg_Info.A = R_A;
    Control_Info->R_Leg_Info.Sip_Leg_Angle  = R_N;
    Control_Info->R_Leg_Info.Sip_Leg_Length = R_t / Control_Info->R_Leg_Info.Biased.K;

    Control_Info->R_Leg_Info.X_J_Dot = (R_A * arm_cos_f32(R_N) * (Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
                                      - R_t * arm_sin_f32(R_N) * (Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->R_Leg_Info.Biased.K);
    Control_Info->R_Leg_Info.Y_J_Dot = (R_A * arm_sin_f32(R_N) * (Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
                                      + R_t * arm_cos_f32(R_N) * (Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->R_Leg_Info.Biased.K);
}

void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info)
{
    /* 左腿 */
    Control_Info->L_Leg_Info.Measure.F  = (Control_Info->L_Leg_Info.Biased.K * (Control_Info->L_Leg_Info.Biased.T_Calf - Control_Info->L_Leg_Info.Biased.T_Thigh)) / Control_Info->L_Leg_Info.A;
    Control_Info->L_Leg_Info.Measure.Tp = (Control_Info->L_Leg_Info.Biased.T_Thigh + Control_Info->L_Leg_Info.Biased.T_Calf);
    /* 右腿 */
    Control_Info->R_Leg_Info.Measure.F  = (Control_Info->R_Leg_Info.Biased.K * (Control_Info->R_Leg_Info.Biased.T_Thigh - Control_Info->R_Leg_Info.Biased.T_Calf)) / Control_Info->R_Leg_Info.A;
    Control_Info->R_Leg_Info.Measure.Tp = (Control_Info->R_Leg_Info.Biased.T_Thigh + Control_Info->R_Leg_Info.Biased.T_Calf);

    if (Control_Info->Chassis_Situation == CHASSIS_BALANCE) {
        Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.Measure.Tp;
        Control_Info->L_Leg_Info.Support.FN = Control_Info->L_Leg_Info.Measure.F * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
                                            + ((Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))
                                               / Control_Info->L_Leg_Info.Sip_Leg_Length);

        Control_Info->R_Leg_Info.Measure.Tp = Control_Info->R_Leg_Info.Measure.Tp;
        Control_Info->R_Leg_Info.Support.FN = Control_Info->R_Leg_Info.Measure.F * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
                                            + (-(Control_Info->R_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))
                                               / Control_Info->R_Leg_Info.Sip_Leg_Length);

        Control_Info->L_Leg_Info.Support.Flag = (Control_Info->L_Leg_Info.Support.FN < 22.f);
        Control_Info->R_Leg_Info.Support.Flag = (Control_Info->R_Leg_Info.Support.FN < 22.f);
    } else {
        Control_Info->L_Leg_Info.Support.Flag = 0;
        Control_Info->R_Leg_Info.Support.Flag = 0;
        Control_Info->L_Leg_Info.Support.FN   = 100.f;
        Control_Info->R_Leg_Info.Support.FN   = 100.f;
    }
}

void VMC_Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info)
{
    float Tourgue_max = 15.f;
    float Current_max = 10000;

    /* 左腿: T2(小腿)=A/K*F + Tp/2, T1(大腿)=-A/K*F + Tp/2 */
    Control_Info->L_Leg_Info.SendValue.T_Calf  = (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F) / Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp / 2.0f);
    Control_Info->L_Leg_Info.SendValue.T_Thigh = (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F) / Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp / 2.0f);

    /* 右腿: T2(大腿)=A/K*F + Tp/2, T1(小腿)=-A/K*F + Tp/2 */
    Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F) / Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp / 2.0f);
    Control_Info->R_Leg_Info.SendValue.T_Calf  = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F) / Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp / 2.0f);

    /* 轮子力矩 → 电流转换 */
    Control_Info->L_Leg_Info.SendValue.Current = (int16_t)(Control_Info->L_Leg_Info.T * 1200.f);
    Control_Info->R_Leg_Info.SendValue.Current = (int16_t)(-Control_Info->R_Leg_Info.T * 1200.f);

    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.Current, -Current_max, Current_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.Current, -Current_max, Current_max);

    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Calf,  -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Thigh, -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Thigh, -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Calf,  -Tourgue_max, Tourgue_max);
}
