/**
  ******************************************************************************
  * @file           : lqr_controller.c
  * @brief          : Gain-scheduled LQR balance controller
  ******************************************************************************
  */

#include "lqr_controller.h"
#include "PID.h"

/* ---- LQR 增益调度多项式系数 ---- */
float K11[6] = {0, -344.130023f,  397.724995f,  -265.059481f,  -4.941964f};
float K12[6] = {0,  11.842778f,  -18.891159f,  -27.922778f,    0.234829f};
float K13[6] = {0, -288.953787f,  281.637253f,  -94.596365f,  -10.720163f};
float K14[6] = {0, -177.996259f,  181.622915f,  -75.159282f,   -7.728459f};
float K15[6] = {0, -835.889683f,  930.198548f,  -389.150660f,   74.543061f};
float K16[6] = {0, -58.542501f,    66.926377f,  -29.456008f,    6.433743f};
float K21[6] = {0,  178.165050f, -120.123702f,   -0.177096f,   29.334646f};
float K22[6] = {0,   38.945329f,  -38.984286f,   14.882355f,    2.371578f};
float K23[6] = {0, -527.320926f,  586.755646f,  -245.391894f,   46.899771f};
float K24[6] = {0, -343.006363f,  380.367381f,  -159.679912f,   32.616099f};
float K25[6] = {0, 1840.588017f, -1794.197881f,  602.816532f,   67.365929f};
float K26[6] = {0,  151.012003f, -149.438347f,   51.534782f,    1.364543f};

/* ---- PID 引用 (定义在 mode_state_machine.c) ---- */
extern PID_Info_TypeDef PID_Leg_Coordinate;

void LQR_K_Update(Control_Info_Typedef *Control_Info)
{
    /* 左腿增益调度: 霍纳法则 */
    float L_L0   = Control_Info->L_Leg_Info.Sip_Leg_Length;
    float L_L0_2 = L_L0 * L_L0;
    float L_L0_3 = L_L0_2 * L_L0;

    Control_Info->L_Leg_Info.LQR_K[0][0] = K11[1]*L_L0_3 + K11[2]*L_L0_2 + K11[3]*L_L0 + K11[4];
    Control_Info->L_Leg_Info.LQR_K[0][1] = K12[1]*L_L0_3 + K12[2]*L_L0_2 + K12[3]*L_L0 + K12[4];
    Control_Info->L_Leg_Info.LQR_K[0][2] = K13[1]*L_L0_3 + K13[2]*L_L0_2 + K13[3]*L_L0 + K13[4];
    Control_Info->L_Leg_Info.LQR_K[0][3] = K14[1]*L_L0_3 + K14[2]*L_L0_2 + K14[3]*L_L0 + K14[4];
    Control_Info->L_Leg_Info.LQR_K[0][4] = K15[1]*L_L0_3 + K15[2]*L_L0_2 + K15[3]*L_L0 + K15[4];
    Control_Info->L_Leg_Info.LQR_K[0][5] = K16[1]*L_L0_3 + K16[2]*L_L0_2 + K16[3]*L_L0 + K16[4];

    Control_Info->L_Leg_Info.LQR_K[1][0] = K21[1]*L_L0_3 + K21[2]*L_L0_2 + K21[3]*L_L0 + K21[4];
    Control_Info->L_Leg_Info.LQR_K[1][1] = K22[1]*L_L0_3 + K22[2]*L_L0_2 + K22[3]*L_L0 + K22[4];
    Control_Info->L_Leg_Info.LQR_K[1][2] = K23[1]*L_L0_3 + K23[2]*L_L0_2 + K23[3]*L_L0 + K23[4];
    Control_Info->L_Leg_Info.LQR_K[1][3] = K24[1]*L_L0_3 + K24[2]*L_L0_2 + K24[3]*L_L0 + K24[4];
    Control_Info->L_Leg_Info.LQR_K[1][4] = K25[1]*L_L0_3 + K25[2]*L_L0_2 + K25[3]*L_L0 + K25[4];
    Control_Info->L_Leg_Info.LQR_K[1][5] = K26[1]*L_L0_3 + K26[2]*L_L0_2 + K26[3]*L_L0 + K26[4];

    /* 右腿增益调度 */
    float R_L0   = Control_Info->R_Leg_Info.Sip_Leg_Length;
    float R_L0_2 = R_L0 * R_L0;
    float R_L0_3 = R_L0_2 * R_L0;

    Control_Info->R_Leg_Info.LQR_K[0][0] = K11[1]*R_L0_3 + K11[2]*R_L0_2 + K11[3]*R_L0 + K11[4];
    Control_Info->R_Leg_Info.LQR_K[0][1] = K12[1]*R_L0_3 + K12[2]*R_L0_2 + K12[3]*R_L0 + K12[4];
    Control_Info->R_Leg_Info.LQR_K[0][2] = K13[1]*R_L0_3 + K13[2]*R_L0_2 + K13[3]*R_L0 + K13[4];
    Control_Info->R_Leg_Info.LQR_K[0][3] = K14[1]*R_L0_3 + K14[2]*R_L0_2 + K14[3]*R_L0 + K14[4];
    Control_Info->R_Leg_Info.LQR_K[0][4] = K15[1]*R_L0_3 + K15[2]*R_L0_2 + K15[3]*R_L0 + K15[4];
    Control_Info->R_Leg_Info.LQR_K[0][5] = K16[1]*R_L0_3 + K16[2]*R_L0_2 + K16[3]*R_L0 + K16[4];

    Control_Info->R_Leg_Info.LQR_K[1][0] = K21[1]*R_L0_3 + K21[2]*R_L0_2 + K21[3]*R_L0 + K21[4];
    Control_Info->R_Leg_Info.LQR_K[1][1] = K22[1]*R_L0_3 + K22[2]*R_L0_2 + K22[3]*R_L0 + K22[4];
    Control_Info->R_Leg_Info.LQR_K[1][2] = K23[1]*R_L0_3 + K23[2]*R_L0_2 + K23[3]*R_L0 + K23[4];
    Control_Info->R_Leg_Info.LQR_K[1][3] = K24[1]*R_L0_3 + K24[2]*R_L0_2 + K24[3]*R_L0 + K24[4];
    Control_Info->R_Leg_Info.LQR_K[1][4] = K25[1]*R_L0_3 + K25[2]*R_L0_2 + K25[3]*R_L0 + K25[4];
    Control_Info->R_Leg_Info.LQR_K[1][5] = K26[1]*R_L0_3 + K26[2]*R_L0_2 + K26[3]*R_L0 + K26[4];
}

void LQR_X_Update(Control_Info_Typedef *Control_Info)
{
    /* 左腿状态误差 */
    Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta            - Control_Info->L_Leg_Info.Measure.Theta);
    Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot        - Control_Info->L_Leg_Info.Measure.Theta_dot);
    Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position - Control_Info->L_Leg_Info.Measure.Chassis_Position);
    Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
    Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi              - Control_Info->L_Leg_Info.Measure.Phi);
    Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot          - Control_Info->L_Leg_Info.Measure.Phi_dot);

    /* 右腿状态误差 */
    Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta            - Control_Info->R_Leg_Info.Measure.Theta);
    Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot        - Control_Info->R_Leg_Info.Measure.Theta_dot);
    Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position);
    Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
    Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi              - Control_Info->R_Leg_Info.Measure.Phi);
    Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot          - Control_Info->R_Leg_Info.Measure.Phi_dot);
}

void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info)
{
    /* 支撑腿自适应增益清零 */
    if (Control_Info->L_Leg_Info.Support.Flag == 1) {
        Control_Info->L_Leg_Info.LQR_K[0][0] = 0;  Control_Info->L_Leg_Info.LQR_K[0][1] = 0;
        Control_Info->L_Leg_Info.LQR_K[0][2] = 0;  Control_Info->L_Leg_Info.LQR_K[0][3] = 0;
        Control_Info->L_Leg_Info.LQR_K[0][4] = 0;  Control_Info->L_Leg_Info.LQR_K[0][5] = 0;
        Control_Info->L_Leg_Info.LQR_K[1][2] = 0;  Control_Info->L_Leg_Info.LQR_K[1][3] = 0;
        Control_Info->L_Leg_Info.LQR_K[1][4] = 0;  Control_Info->L_Leg_Info.LQR_K[1][5] = 0;
    }
    if (Control_Info->R_Leg_Info.Support.Flag == 1) {
        Control_Info->R_Leg_Info.LQR_K[0][0] = 0;  Control_Info->R_Leg_Info.LQR_K[0][1] = 0;
        Control_Info->R_Leg_Info.LQR_K[0][2] = 0;  Control_Info->R_Leg_Info.LQR_K[0][3] = 0;
        Control_Info->R_Leg_Info.LQR_K[0][4] = 0;  Control_Info->R_Leg_Info.LQR_K[0][5] = 0;
        Control_Info->R_Leg_Info.LQR_K[1][2] = 0;  Control_Info->R_Leg_Info.LQR_K[1][3] = 0;
        Control_Info->R_Leg_Info.LQR_K[1][4] = 0;  Control_Info->R_Leg_Info.LQR_K[1][5] = 0;
    }

    /* u = K·X 左腿 — 力矩T (轮子) */
    Control_Info->L_Leg_Info.Moment.Balance_T = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.LQR_Output[0][j] = Control_Info->L_Leg_Info.LQR_X[j] * Control_Info->L_Leg_Info.LQR_K[0][j];
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.Moment.Balance_T += Control_Info->L_Leg_Info.LQR_Output[0][j];

    /* 右腿 — 力矩T */
    Control_Info->R_Leg_Info.Moment.Balance_T = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.LQR_Output[0][j] = Control_Info->R_Leg_Info.LQR_X[j] * Control_Info->R_Leg_Info.LQR_K[0][j];
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.Moment.Balance_T += Control_Info->R_Leg_Info.LQR_Output[0][j];

    /* 左腿 — 扭矩Tp (关节) */
    Control_Info->L_Leg_Info.Moment.Balance_Tp = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.LQR_Output[1][j] = Control_Info->L_Leg_Info.LQR_X[j] * Control_Info->L_Leg_Info.LQR_K[1][j];
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.Moment.Balance_Tp += Control_Info->L_Leg_Info.LQR_Output[1][j];
    Control_Info->L_Leg_Info.Moment.Balance_Tp = -Control_Info->L_Leg_Info.Moment.Balance_Tp;

    /* 右腿 — 扭矩Tp */
    Control_Info->R_Leg_Info.Moment.Balance_Tp = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.LQR_Output[1][j] = Control_Info->R_Leg_Info.LQR_X[j] * Control_Info->R_Leg_Info.LQR_K[1][j];
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.Moment.Balance_Tp += Control_Info->R_Leg_Info.LQR_Output[1][j];

    /* 防劈叉 PID + 综合力矩 */
    PID_Calculate(&PID_Leg_Coordinate, 0, Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);
    Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
    Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;

    /* 力矩综合 */
    Control_Info->L_Leg_Info.T = Control_Info->L_Leg_Info.Moment.Balance_T + Control_Info->L_Leg_Info.Moment.Turn_T;
    Control_Info->R_Leg_Info.T = Control_Info->R_Leg_Info.Moment.Balance_T + Control_Info->R_Leg_Info.Moment.Turn_T;
    Control_Info->L_Leg_Info.Tp = Control_Info->L_Leg_Info.Moment.Balance_Tp + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
    Control_Info->R_Leg_Info.Tp = Control_Info->R_Leg_Info.Moment.Balance_Tp + Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;

    /* 支撑腿 / CHASSIS_WEAK 安全清零 */
    if (Control_Info->L_Leg_Info.Support.Flag == 1) Control_Info->L_Leg_Info.T = 0;
    if (Control_Info->R_Leg_Info.Support.Flag == 1) Control_Info->R_Leg_Info.T = 0;
    if (Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        Control_Info->L_Leg_Info.Tp = 0;  Control_Info->R_Leg_Info.Tp = 0;
        Control_Info->L_Leg_Info.F  = 0;  Control_Info->R_Leg_Info.F  = 0;
        Control_Info->L_Leg_Info.T  = 0;  Control_Info->R_Leg_Info.T  = 0;
    }
}
