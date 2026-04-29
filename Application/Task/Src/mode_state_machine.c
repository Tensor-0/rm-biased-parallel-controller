/**
  ******************************************************************************
  * @file           : mode_state_machine.c
  * @brief          : System init, low-voltage check, mode state machine
  ******************************************************************************
  */

#include "mode_state_machine.h"
#include "PID.h"

/* ---- PID parameters (KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput) ---- */
#define PID_PARAM(Kp,Ki,Kd,A,Db,Li,Lo) { (Kp), (Ki), (Kd), (A), (Db), (Li), (Lo) }

static float PID_Leg_Length_F_Param[7]  = PID_PARAM(1300.f, 1.f,  60000.f, 0.f, 0.f, 10.f, 100.f);
static float PID_Leg_Roll_F_Param[7]    = PID_PARAM(50.f,   0.f,  25.f,    0.f, 0.f, 0.1f, 50.f);
static float PID_Leg_Coordinate_param[7]= PID_PARAM(300.f,  0.f,  20.0f,  0.f, 0.f, 0.f,  50);
static float PID_Yaw_P_pama[7]          = PID_PARAM(4.4f,   0.f,  60.f,   0,   0,   200,  500);
static float PID_Yaw_V_pama[7]          = PID_PARAM(0.25f,  0,    0.4f,   0,   0,   200,  70);

/* ---- PID instances ---- */
PID_Info_TypeDef PID_Leg_Coordinate;
PID_Info_TypeDef PID_Leg_length_F[2];
PID_Info_TypeDef PID_Leg_Roll_F;
PID_Info_TypeDef PID_Yaw[2];

void Mode_Init(Control_Info_Typedef *Control_Info)
{
    PID_Init(&PID_Leg_length_F[0], PID_POSITION, PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_length_F[1], PID_POSITION, PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_Roll_F,      PID_POSITION, PID_Leg_Roll_F_Param);
    PID_Init(&PID_Leg_Coordinate,  PID_POSITION, PID_Leg_Coordinate_param);
    PID_Init(&PID_Yaw[0],          PID_POSITION, PID_Yaw_P_pama);
    PID_Init(&PID_Yaw[1],          PID_POSITION, PID_Yaw_V_pama);
}

void Mode_Check_Low_Voltage(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    Control_Info->VDC = in->vbat;
}

void Mode_Update(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    if (in->rc.s[1] == 3 || in->rc.s[1]) {
        Control_Info->Init.IF_Begin_Init = 1;
        if (in->rc.s[1] == 2) {
            Control_Info->Init.IF_Begin_Init = 0;
            Control_Info->Chassis_Situation = CHASSIS_WEAK;
        }
    } else {
        Control_Info->Init.IF_Begin_Init = 0;
        if (Control_Info->Chassis_Situation == CHASSIS_BALANCE)
            Control_Info->Chassis_Situation = CHASSIS_WEAK;
    }

    if (Control_Info->Init.IF_Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        if (Control_Info->Init.Joint_Init.IF_Joint_Init == 0) {
            if (in->joint[0].position < 0.0f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
            if (in->joint[1].position > -0.21f && in->joint[1].position < -0.005f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;
            if (in->joint[2].position < 0.20f && in->joint[2].position > 0.01f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
            if (in->joint[3].position > -0.0f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;

            if (Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] == 4) {
                Control_Info->Init.Joint_Init.IF_Joint_Init = 1;
            } else {
                Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
            }
            Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
            Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
        } else if (Control_Info->Init.Joint_Init.IF_Joint_Init == 1) {
            Control_Info->Chassis_Situation = CHASSIS_BALANCE;
            Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
            Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
        }
    }

    if (Control_Info->Init.IF_Begin_Init == 0 && Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
    }
}
