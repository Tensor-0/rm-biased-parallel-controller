/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @brief          : IMU + wheel odometry sensor fusion
  ******************************************************************************
  */

#include "sensor_fusion.h"
#include "arm_math.h"

#define GravityAccel 9.81f

void SensorFusion_Measure_Update(Control_Info_Typedef *Control_Info,
                                  const control_input_snapshot_t *in)
{
    /* --- 共享 IMU 数据 (避免重复读取) --- */
    float phi_shared     = -in->ins.Angle[2] - Control_Info->L_Leg_Info.Phi_Comp_Angle;
    float phi_dot_shared = -in->ins.Gyro[0];

    /* --- 左腿姿态 --- */
    Control_Info->L_Leg_Info.Measure.Phi     = phi_shared;
    Control_Info->L_Leg_Info.Measure.Phi_dot = phi_dot_shared;
    Control_Info->L_Leg_Info.Measure.Theta   = ((PI/2) - Control_Info->L_Leg_Info.Sip_Leg_Angle) - Control_Info->L_Leg_Info.Measure.Phi;
    Control_Info->L_Leg_Info.Measure.Theta_dot = (Control_Info->L_Leg_Info.X_J_Dot * arm_cos_f32(-Control_Info->L_Leg_Info.Measure.Theta)
                                                + Control_Info->L_Leg_Info.Y_J_Dot * arm_sin_f32(-Control_Info->L_Leg_Info.Measure.Theta))
                                               / Control_Info->L_Leg_Info.Sip_Leg_Length;

    /* --- 右腿姿态 --- */
    Control_Info->R_Leg_Info.Measure.Phi     = phi_shared;
    Control_Info->R_Leg_Info.Measure.Phi_dot = phi_dot_shared;
    Control_Info->R_Leg_Info.Measure.Theta   = (Control_Info->R_Leg_Info.Sip_Leg_Angle - (PI/2)) - Control_Info->R_Leg_Info.Measure.Phi;
    Control_Info->R_Leg_Info.Measure.Theta_dot = (-Control_Info->R_Leg_Info.X_J_Dot * arm_cos_f32(-Control_Info->R_Leg_Info.Measure.Theta)
                                                 + Control_Info->R_Leg_Info.Y_J_Dot * arm_sin_f32(-Control_Info->R_Leg_Info.Measure.Theta))
                                                / Control_Info->R_Leg_Info.Sip_Leg_Length;

    /* --- 左腿轮速融合 --- */
    Control_Info->L_Leg_Info.Velocity.Wheel = in->wheel[0].velocity * (PI / 30.f) / 15.f;
    Control_Info->L_Leg_Info.Velocity.W     = (Control_Info->L_Leg_Info.Velocity.Wheel
                                              - Control_Info->L_Leg_Info.Measure.Phi_dot
                                              + Control_Info->L_Leg_Info.Measure.Theta_dot);
    Control_Info->L_Leg_Info.Velocity.X     = Control_Info->L_Leg_Info.Velocity.W * 0.055f;
    Control_Info->L_Leg_Info.Sip_Leg_Length_dot = Control_Info->L_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta);
    Control_Info->L_Leg_Info.Velocity.Body  = Control_Info->L_Leg_Info.Velocity.X;
    Control_Info->L_Leg_Info.Predict_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion + Control_Info->Accel * 0.001f;
    Control_Info->L_Leg_Info.Velocity.Fusion  = Control_Info->L_Leg_Info.Velocity.Predict * 0.8f + Control_Info->L_Leg_Info.Velocity.Body * 0.2f;
    Control_Info->L_Leg_Info.Measure.Chassis_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion;

    /* --- 右腿轮速融合 --- */
    Control_Info->R_Leg_Info.Velocity.Wheel = -in->wheel[1].velocity * (3.141593f / 30.f) / 15.f;
    Control_Info->R_Leg_Info.Velocity.W     = (Control_Info->R_Leg_Info.Velocity.Wheel
                                              - Control_Info->R_Leg_Info.Measure.Phi_dot
                                              + Control_Info->R_Leg_Info.Measure.Theta_dot);
    Control_Info->R_Leg_Info.Velocity.X     = Control_Info->R_Leg_Info.Velocity.W * 0.055f;
    Control_Info->R_Leg_Info.Sip_Leg_Length_dot = Control_Info->R_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta);
    Control_Info->R_Leg_Info.Velocity.Body  = Control_Info->R_Leg_Info.Velocity.X;
    Control_Info->R_Leg_Info.Velocity.Predict = Control_Info->R_Leg_Info.Velocity.Fusion + Control_Info->Accel * 0.001f;
    Control_Info->R_Leg_Info.Velocity.Fusion  = 0.8f * Control_Info->R_Leg_Info.Velocity.Predict + Control_Info->R_Leg_Info.Velocity.Body * 0.2f;
    Control_Info->R_Leg_Info.Measure.Chassis_Velocity = Control_Info->R_Leg_Info.Velocity.Fusion;

    /* --- 底盘速度与位置 --- */
    Control_Info->Chassis_Velocity = (Control_Info->L_Leg_Info.Measure.Chassis_Velocity + Control_Info->R_Leg_Info.Measure.Chassis_Velocity) / 2.f;

    if (Control_Info->Target_Velocity == 0) {
        Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity * 0.001f;
        Control_Info->R_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity * 0.001f;
    } else {
        Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
    }

    /* --- 底盘加速度 --- */
    Control_Info->Accel = (float)((-in->ins.Accel[1] + powf(in->ins.Gyro[2], 2) * 0.155f)
                          - GravityAccel * arm_sin_f32(-in->ins.Angle[2])) * arm_cos_f32(-in->ins.Angle[2])
                        + (in->ins.Accel[2] - GravityAccel * arm_cos_f32(-in->ins.Angle[2])) * arm_sin_f32(-in->ins.Angle[2]);

    /* --- CHASSIS_WEAK 状态清零 --- */
    if (Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        Control_Info->L_Leg_Info.Measure.Phi = 0;
        Control_Info->L_Leg_Info.Measure.Phi_dot = 0;
        Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->L_Leg_Info.Measure.Chassis_Velocity = 0;
        Control_Info->L_Leg_Info.Measure.Theta = 0;
        Control_Info->L_Leg_Info.Measure.Theta_dot = 0;

        Control_Info->R_Leg_Info.Measure.Phi = 0;
        Control_Info->R_Leg_Info.Measure.Phi_dot = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Velocity = 0;
        Control_Info->R_Leg_Info.Measure.Theta = 0;
        Control_Info->R_Leg_Info.Measure.Theta_dot = 0;

        Control_Info->Chassis_Velocity = 0;
    }
}
