/**
  ******************************************************************************
  * @file           : lqr_controller.c
  * @brief          : Gain-scheduled LQR: cubic polynomial K(L₀) via Horner, u = −Kx, adaptive zeroing.
  *
  * ======  新手必读 ======
  *
  * 【这个文件在干什么？】
  *   假设你现在站歪了,你想回到直立。你需要知道两件事:
  *   1. 我现在差目标状态多少? (X = Target - Measure, 这就是 LQR_X_Update)
  *   2. 差这么多该用多大力纠正? (K_optimal, 这就是 LQR_K_Update)
  *   然后用 u = -K·X 得到最优输出(LQR_T_Tp_Calculate)。
  *
  *   但 K 不是固定的——腿越长,杠杆效应越大,需要的力越小。
  *   所以 K = f(L₀) 是一个关于腿长的三次多项式,这叫"增益调度"。
  *
  * 【核心职责】
  *   1. LQR_K_Update:    根据当前虚拟腿长查多项式 → 得到最优增益 K[2][6]
  *   2. LQR_X_Update:    计算 6 维状态误差向量
  *   3. LQR_T_Tp_Calculate: u = -K·X → 力矩 T + 扭矩 Tp + 综合力矩
  *
  * 【前置知识】
  *   - 6维状态向量: [θ误差, dθ误差, x误差, v误差, φ误差, dφ误差]
  *   - LQR 基本公式: u = -K·x (K由离线 MATLAB 算出)
  *   - 霍纳法则: 多项式 K(L₀) = c1·L₀³+c2·L₀²+c3·L₀+c4 的快速计算法
  *
  ******************************************************************************
  */

#include "lqr_controller.h"
#include "PID.h"

/* ====== LQR 增益调度的多项式系数 ======
 *
 *  这些系数是提前在MATLAB中用 lqr() 函数算出来的。
 *   MATLAB 脚本放在项目根目录的 get_K.m 中。
 *
 *   用法: 对于第i行第j列的增益 Kij,
 *     Kij(L₀) = Kij[1]×L₀³ + Kij[2]×L₀² + Kij[3]×L₀ + Kij[4]
 *
 *   每个数组有 6 个元素: [0号占位, c1(L₀³), c2(L₀²), c3(L₀), c4(常数)]
 *   为什么第一个元素是 0？因为 Kij[0] 预留给"按需求添加 L₀⁴ 项"
 *   用霍纳法则计算时,实际上是:
 *     K = ((Kij[1]×L₀ + Kij[2])×L₀ + Kij[3])×L₀ + Kij[4]
 *   但我们直接展开为: Kij[1]×L₀³ + Kij[2]×L₀² + Kij[3]×L₀ + Kij[4]
 */

/* 第1行(力矩T)的系数: K1j 控制轮子输出力矩 */
float K11[6] = {0, -344.130023f,  397.724995f,  -265.059481f,  -4.941964f};
float K12[6] = {0,  11.842778f,  -18.891159f,  -27.922778f,    0.234829f};
float K13[6] = {0, -288.953787f,  281.637253f,  -94.596365f,  -10.720163f};
float K14[6] = {0, -177.996259f,  181.622915f,  -75.159282f,   -7.728459f};
float K15[6] = {0, -835.889683f,  930.198548f,  -389.150660f,   74.543061f};
float K16[6] = {0, -58.542501f,    66.926377f,  -29.456008f,    6.433743f};

/* 第2行(扭矩Tp)的系数: K2j 控制关节输出扭矩 */
float K21[6] = {0,  178.165050f, -120.123702f,   -0.177096f,   29.334646f};
float K22[6] = {0,   38.945329f,  -38.984286f,   14.882355f,    2.371578f};
float K23[6] = {0, -527.320926f,  586.755646f,  -245.391894f,   46.899771f};
float K24[6] = {0, -343.006363f,  380.367381f,  -159.679912f,   32.616099f};
float K25[6] = {0, 1840.588017f, -1794.197881f,  602.816532f,   67.365929f};
float K26[6] = {0,  151.012003f, -149.438347f,   51.534782f,    1.364543f};

extern PID_Info_TypeDef PID_Leg_Coordinate;

/**
 * LQR_K_Update — 增益调度: 根据腿长更新 K 矩阵
 *
 *  为什么 K 矩阵要随着腿长变？
 *   想象你用一根长棍子和一根短棍子撑地:
 *   - 长棍子: 轻轻用力就能改变倾角 (力矩臂长 → 增益小)
 *   - 短棍子: 要用很大力才能改变倾角 (力矩臂短 → 增益大)
 *   所以 LQR 的增益必须"自适应"腿长: K = f(L₀)
 *
 *   使用的多项式:
 *     K(L₀) = K[1]×L₀³ + K[2]×L₀² + K[3]×L₀ + K[4]
 *   用霍纳法则快速计算,避免调用慢速 powf()。
 *
 *   左右腿各需要计算 2×6=12 个 K 元素。
 */
void LQR_K_Update(Control_Info_Typedef *Control_Info)
{
    /* ====== LEFT LEG 左腿增益调度 ====== */
    float L_L0   = Control_Info->L_Leg_Info.Sip_Leg_Length;  /* 虚拟腿长 */
    float L_L0_2 = L_L0 * L_L0;   /* L₀² */
    float L_L0_3 = L_L0_2 * L_L0; /* L₀³ */

    /* 第1行: 当状态向量 X[i] 偏离目标时,应该给轮子多大的力矩 T？
       K11~K16 分别对应 6 个状态维度 */
    Control_Info->L_Leg_Info.LQR_K[0][0] = K11[1]*L_L0_3 + K11[2]*L_L0_2 + K11[3]*L_L0 + K11[4];
    Control_Info->L_Leg_Info.LQR_K[0][1] = K12[1]*L_L0_3 + K12[2]*L_L0_2 + K12[3]*L_L0 + K12[4];
    Control_Info->L_Leg_Info.LQR_K[0][2] = K13[1]*L_L0_3 + K13[2]*L_L0_2 + K13[3]*L_L0 + K13[4];
    Control_Info->L_Leg_Info.LQR_K[0][3] = K14[1]*L_L0_3 + K14[2]*L_L0_2 + K14[3]*L_L0 + K14[4];
    Control_Info->L_Leg_Info.LQR_K[0][4] = K15[1]*L_L0_3 + K15[2]*L_L0_2 + K15[3]*L_L0 + K15[4];
    Control_Info->L_Leg_Info.LQR_K[0][5] = K16[1]*L_L0_3 + K16[2]*L_L0_2 + K16[3]*L_L0 + K16[4];

    /* 第2行: 当状态向量 X[i] 偏离目标时,应该给关节多大的扭矩 Tp？ */
    Control_Info->L_Leg_Info.LQR_K[1][0] = K21[1]*L_L0_3 + K21[2]*L_L0_2 + K21[3]*L_L0 + K21[4];
    Control_Info->L_Leg_Info.LQR_K[1][1] = K22[1]*L_L0_3 + K22[2]*L_L0_2 + K22[3]*L_L0 + K22[4];
    Control_Info->L_Leg_Info.LQR_K[1][2] = K23[1]*L_L0_3 + K23[2]*L_L0_2 + K23[3]*L_L0 + K23[4];
    Control_Info->L_Leg_Info.LQR_K[1][3] = K24[1]*L_L0_3 + K24[2]*L_L0_2 + K24[3]*L_L0 + K24[4];
    Control_Info->L_Leg_Info.LQR_K[1][4] = K25[1]*L_L0_3 + K25[2]*L_L0_2 + K25[3]*L_L0 + K25[4];
    Control_Info->L_Leg_Info.LQR_K[1][5] = K26[1]*L_L0_3 + K26[2]*L_L0_2 + K26[3]*L_L0 + K26[4];

    /* ====== RIGHT LEG 右腿增益调度 (使用相同系数,不同的腿长) ====== */
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

/**
 * LQR_X_Update — 计算 6 维状态误差向量
 *
 *  X[i] = 目标值 - 测量值
 *
 *   6个维度的物理含义:
 *   X[0] = 虚拟腿倾角误差     (rad)    — 腿偏离竖直方向多少
 *   X[1] = 虚拟腿倾角速度误差 (rad/s)  — 腿正在倒/正在立
 *   X[2] = 底盘位置误差       (m)      — 机器人偏离目标位置
 *   X[3] = 底盘速度误差       (m/s)    — 机器人正在移动多快
 *   X[4] = 机身倾角误差       (rad)    — 机身偏离水平多少
 *   X[5] = 机身倾角速度误差   (rad/s)  — 机身正在倒多快
 *
 *   目标值大部分是 0: "我希望不倾斜,不偏移,不移动"
 */
void LQR_X_Update(Control_Info_Typedef *Control_Info)
{
    /* ====== LEFT LEG ====== */
    Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta            - Control_Info->L_Leg_Info.Measure.Theta);
    Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot        - Control_Info->L_Leg_Info.Measure.Theta_dot);
    Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position - Control_Info->L_Leg_Info.Measure.Chassis_Position);
    Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
    Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi              - Control_Info->L_Leg_Info.Measure.Phi);
    Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot          - Control_Info->L_Leg_Info.Measure.Phi_dot);

    /* ====== RIGHT LEG ====== */
    Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta            - Control_Info->R_Leg_Info.Measure.Theta);
    Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot        - Control_Info->R_Leg_Info.Measure.Theta_dot);
    Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position);
    Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
    Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi              - Control_Info->R_Leg_Info.Measure.Phi);
    Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot          - Control_Info->R_Leg_Info.Measure.Phi_dot);
}

/**
 * LQR_T_Tp_Calculate — LQR 最优控制 + 自适应 + 综合力矩
 *
 *  这个函数是整个系统的"最终决策":
 *   1. 着地自适应: 空中的腿 → 禁用水平推力(蹬不到地)
 *   2. u = K·X: 增益矩阵×状态误差 = 最优控制力
 *   3. 防劈叉 PID: 两腿角度差不要太大
 *   4. 力矩综合: Balance + Turn → T, Balance + Leg_Coordinate → Tp
 *   5. 安全清零: 支撑腿/虚弱状态下清掉不合理输出
 */
void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info)
{
    /* ====== 步骤1: 着地自适应 —— 松掉空中腿的增益 ======
        如果这条腿在空中(Support.Flag==1),轮子空转是浪费能量,
       且空转产生的反作用力会干扰平衡。所以把 K[0][*]全部清零(禁用轮子前推力)
       K[1][2:5]清零(部分禁用关节扭矩中与位置/速度/倾角相关的项) */
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

    /* ====== 步骤2: u = K·X —— 左腿力矩 T (轮子) ====== */
    Control_Info->L_Leg_Info.Moment.Balance_T = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.LQR_Output[0][j] = Control_Info->L_Leg_Info.LQR_X[j] * Control_Info->L_Leg_Info.LQR_K[0][j];
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.Moment.Balance_T += Control_Info->L_Leg_Info.LQR_Output[0][j];

    /* 右腿力矩 T (轮子) */
    Control_Info->R_Leg_Info.Moment.Balance_T = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.LQR_Output[0][j] = Control_Info->R_Leg_Info.LQR_X[j] * Control_Info->R_Leg_Info.LQR_K[0][j];
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.Moment.Balance_T += Control_Info->R_Leg_Info.LQR_Output[0][j];

    /* ====== 步骤3: u = K·X —— 左腿扭矩 Tp (关节) ====== */
    Control_Info->L_Leg_Info.Moment.Balance_Tp = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.LQR_Output[1][j] = Control_Info->L_Leg_Info.LQR_X[j] * Control_Info->L_Leg_Info.LQR_K[1][j];
    for (int j = 0; j < 6; j++)
        Control_Info->L_Leg_Info.Moment.Balance_Tp += Control_Info->L_Leg_Info.LQR_Output[1][j];
    Control_Info->L_Leg_Info.Moment.Balance_Tp = -Control_Info->L_Leg_Info.Moment.Balance_Tp; /* 左腿符号取反 */

    /* 右腿扭矩 Tp (关节) */
    Control_Info->R_Leg_Info.Moment.Balance_Tp = 0;
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.LQR_Output[1][j] = Control_Info->R_Leg_Info.LQR_X[j] * Control_Info->R_Leg_Info.LQR_K[1][j];
    for (int j = 0; j < 6; j++)
        Control_Info->R_Leg_Info.Moment.Balance_Tp += Control_Info->R_Leg_Info.LQR_Output[1][j];

    /* ====== 步骤4: 防劈叉 PID ======
        如果两条腿的虚拟腿倾角差太多,机器人会"劈叉"。
       这个 PID 专门产生一个同步扭矩,把两腿的倾角往回拉。
       目标=0(希望角度差为0), 测量=左腿Theta - 右腿Theta */
    PID_Calculate(&PID_Leg_Coordinate, 0, Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);
    Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
    Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;

    /* ====== 步骤5: 力矩综合 ======
        把各通道的力矩"加在一起"变成最终输出:
       T  = Balance_T(平衡) + Turn_T(转向)
       Tp = Balance_Tp(平衡扭矩) + Leg_Coordinate_Tp(防劈叉) */
    Control_Info->L_Leg_Info.T = Control_Info->L_Leg_Info.Moment.Balance_T + Control_Info->L_Leg_Info.Moment.Turn_T;
    Control_Info->R_Leg_Info.T = Control_Info->R_Leg_Info.Moment.Balance_T + Control_Info->R_Leg_Info.Moment.Turn_T;
    Control_Info->L_Leg_Info.Tp = Control_Info->L_Leg_Info.Moment.Balance_Tp + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
    Control_Info->R_Leg_Info.Tp = Control_Info->R_Leg_Info.Moment.Balance_Tp + Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;

    /* ====== ⚠ 安全清零 ======
        如果不加这些 if,会出现以下危险:
       - 空中的支撑腿还在输出前推力 → 落地瞬间会暴冲
       - CHASSIS_WEAK 状态下 LQR 还在计算 → 可能产生超大控制量 */
    if (Control_Info->L_Leg_Info.Support.Flag == 1) Control_Info->L_Leg_Info.T = 0;
    if (Control_Info->R_Leg_Info.Support.Flag == 1) Control_Info->R_Leg_Info.T = 0;
    if (Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        /*  虚弱状态下所有力和力矩清零 —— "安全气囊" */
        Control_Info->L_Leg_Info.Tp = 0;  Control_Info->R_Leg_Info.Tp = 0;
        Control_Info->L_Leg_Info.F  = 0;  Control_Info->R_Leg_Info.F  = 0;
        Control_Info->L_Leg_Info.T  = 0;  Control_Info->R_Leg_Info.T  = 0;
    }
}
