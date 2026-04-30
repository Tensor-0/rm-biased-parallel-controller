/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : 惯性导航系统 — 把 BMI088 的陀螺仪+加速度计数据变成机身姿态
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   机器人身上挂着一个 BMI088 六轴 IMU 芯片。这个芯片每时每刻在告诉你:
  *   - 角速度(Gyro): 你在绕哪个轴转多快 (°/s 或 rad/s)
  *   - 加速度(Accel): 你在哪个方向承受多少 G 力 (m/s²)
  *   但这个数据很"原始"——有噪声、有漂移。本文件的任务:
  *   1. 读原始数据 → 做低通滤波(去毛刺)
  *   2. 跑四元数 EKF 算法 → 姿态角(欧拉角)
  *   3. 维护偏航角的"总圈数"(转超过360°不进位)
  *   4. 控制 IMU 温度(用 PWM 加热)
  *
  * 【核心职责】
  *   1. 读取 BMI088 原始数据
  *   2. 二阶低通滤波 (加速度)
  *   3. 四元数 EKF 姿态解算 → 欧拉角 Pitch/Yaw/Roll
  *   4. 偏航总圈数累计
  *   5. BMI088 温度 PID 控制
  *
  * 【前置知识】
  *   - 欧拉角: Pitch=前后倾, Yaw=左右转, Roll=侧翻
  *   - 四元数 EKF: 一个融合陀螺仪和加速度计的算法,让姿态既快又准
  *   - 低通滤波: 把高频毛刺"磨平",但保留低频的有用信号
  *   - 为什么要温度控制? BMI088 温度变化会影响零偏(传感器读的0不一定是真0)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "Bmi088.h"
#include "LPF.h"
#include "PID.h"
#include "Config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

/* ====== 全局 IMU 状态 ======
   🐣 INS_Info 是整个系统的"陀螺仪数据仓库"。
   INS_Task 写入, Control_Task 通过快照读取。 */
INS_Info_Typedef INS_Info;

/* ====== 二阶低通滤波器系数 ======
   🐣 这三个系数决定了滤波器"有多磨"。值从 MATLAB 的 butter/filter designer 算出来。
   不需要理解每个数字——你只需要知道: "加速度的毛刺被磨平了,姿态就不会抖"。 */
static float INS_LPF2p_Alpha[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/* 3 个二阶低通滤波器实例: 分别滤波 X/Y/Z 三轴的加速度 */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];

/* ====== EKF 初始化矩阵 ====== */
/* [状态转移矩阵 A] 初始为单位矩阵(6×6) */
static float QuaternionEKF_A_Data[36] = {1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0,
                                         0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
/* [后验协方差矩阵 P] 最初设很大的值,表示"我还很不确定" */
static float QuaternionEKF_P_Data[36] = {100000,0.1,0.1,0.1,0.1,0.1, 0.1,100000,0.1,0.1,0.1,0.1,
                                         0.1,0.1,100000,0.1,0.1,0.1, 0.1,0.1,0.1,100000,0.1,0.1,
                                         0.1,0.1,0.1,0.1,100,0.1, 0.1,0.1,0.1,0.1,0.1,100};

/* ====== BMI088 温度控制 PID 参数 ====== */
static float TemCtrl_PID_Param[7] = {1200, 20, 0, 0, 0, 0, 2000};
PID_Info_TypeDef TempCtrl_PID;

/* 前向声明 */
static void INS_Task_Init(void);
static void BMI088_Temp_Control(float temp);

/**
 * INS_Task — IMU 姿态解算线程 (1kHz)
 *
 * 🐣 每 1ms 执行一次完整的姿态解算流程:
 *   1. 读 BMI088 原始数据 (SPI 通信)
 *   2. 加速度做二阶低通滤波 (去除高频噪声)
 *   3. 四元数 EKF 更新 (融合陀螺仪和加速度 → 最优姿态)
 *   4. 更新欧拉角 (弧度 + 度)
 *   5. 偏航总圈数维护 (防止 Yaw 从 180° 突变到 -180°)
 *   6. 每 5 次循环执行一次温度控制 (200Hz)
 */
void INS_Task(void const * argument)
{
    TickType_t INS_Task_SysTick = 0;

    /* 初始化: 低通滤波器 + 温度PID + EKF */
    INS_Task_Init();

    for (;;) {
        INS_Task_SysTick = osKernelSysTick();

        /* 🐣 步骤1: 读取 BMI088 原始数据
           SPI 通信读取陀螺仪( °/s )和加速度计( m/s² )
           数据保存在 BMI088_Info 全局结构体中 */
        BMI088_Info_Update(&BMI088_Info);

        /* 🐣 步骤2: 加速度做二阶低通滤波
           为什么只滤波加速度？加速度对振动非常敏感(走路时上下抖动),
           而陀螺仪天生比较平滑——滤波后加速度的"假运动"被滤掉,
           EKF 不会误以为"车在加速"而去错误修正姿态。 */
        INS_Info.Accel[0] = LowPassFilter2p_Update(&INS_AccelPF2p[0], BMI088_Info.Accel[0]);
        INS_Info.Accel[1] = LowPassFilter2p_Update(&INS_AccelPF2p[1], BMI088_Info.Accel[1]);
        INS_Info.Accel[2] = LowPassFilter2p_Update(&INS_AccelPF2p[2], BMI088_Info.Accel[2]);

        /* 🐣 步骤3: 更新陀螺仪数据 (弧度/秒)
           从 BMI088 原始数据(°/s)转换为弧度/秒 */
        INS_Info.Gyro[0] = BMI088_Info.Gyro[0];
        INS_Info.Gyro[1] = BMI088_Info.Gyro[1];
        INS_Info.Gyro[2] = BMI088_Info.Gyro[2];

        /* 🐣 步骤4: 四元数 EKF 更新
           QuaternionEKF_Update 完成:
           - 利用角速度预测下一时刻的四元数 (陀螺仪积分)
           - 利用加速度修正四元数 (重力方向观测)
           - 输出欧拉角 EulerAngle[3] (弧度) */
        QuaternionEKF_Update(&Quaternion_Info, INS_Info.Gyro, INS_Info.Accel, 0.001f);

        /* 🐣 步骤5: 拷贝欧拉角到 INS_Info
           Angle[0]=Roll, Angle[1]=Pitch, Angle[2]=Yaw */
        memcpy(INS_Info.Angle, Quaternion_Info.EulerAngle, sizeof(INS_Info.Angle));

        /* 🐣 步骤6: 弧度 → 角度转换
           57.295779513f = 180/π */
        INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH] * 57.295779513f;
        INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   * 57.295779513f;
        INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]  * 57.295779513f;

        /* 🐣 步骤7: 偏航总圈数维护
           Yaw 输出的范围是 [-180°, 180°],当你转过 180° 时会跳到 -180°。
           通过比较相邻两次的 Yaw 角,判断是否跨过了 ±180° 边界,
           跨过了就 ±1 圈数。Yaw_TolAngle = 累计角度 = Yaw_Angle + 圈数×360°。
           这样连续转 10 圈的累计角度是 3600°,而不是在 ±180° 之间来回跳。 */
        if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f) {
            INS_Info.YawRoundCount++;  /* 顺时针跨过+180° → 圈数+1 */
        }
        else if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f) {
            INS_Info.YawRoundCount--;  /* 逆时针跨过-180° → 圈数-1 */
        }
        INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
        INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.f;

        /* 🐣 步骤8: 角速度也转成"度/秒" (Control_Task 偏航PID 用) */
        INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH] * RadiansToDegrees;
        INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]   * RadiansToDegrees;
        INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]  * RadiansToDegrees;

        /* 🐣 步骤9: 每5个周期(200Hz)执行一次温度控制
           温度控制的频率比姿态解算低,因为温度变化本身很慢 */
        if (INS_Task_SysTick % 5 == 0) {
            BMI088_Temp_Control(BMI088_Info.Temperature);
        }

        /* 🐣 等待 1ms (保证 1kHz 精确频率) */
        osDelayUntil(&INS_Task_SysTick, 1);
    }
}

/**
 * INS_Task_Init — 初始化低通滤波器、温度PID、四元数EKF
 *
 * 🐣 在 INS_Task 启动时只跑一次。
 */
static void INS_Task_Init(void)
{
    /* 初始化 3 个二阶低通滤波器(X/Y/Z轴) */
    LowPassFilter2p_Init(&INS_AccelPF2p[0], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[1], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[2], INS_LPF2p_Alpha);

    /* 初始化温度控制 PID */
    PID_Init(&TempCtrl_PID, PID_POSITION, TemCtrl_PID_Param);

    /* 初始化四元数 EKF
       参数: 过程噪声10, dt=0.001s, 观测噪声1000000, 初始A/P矩阵 */
    QuaternionEKF_Init(&Quaternion_Info, 10.f, 0.001f, 1000000.f, QuaternionEKF_A_Data, QuaternionEKF_P_Data);
}

/**
 * BMI088_Temp_Control — BMI088 温度控制
 *
 * 🐣 BMI088 的工作温度会影响其零偏(读数偏置)。稳定在 40°C 左右,数据最准。
 *   PID 计算加热功率 → 通过 PWM 控制加热电阻。
 */
static void BMI088_Temp_Control(float Temp)
{
    /* 目标温度 40°C, PID 计算加热功率 */
    PID_Calculate(&TempCtrl_PID, 40.f, Temp);

    /* ⚠ 限幅: 加热功率不能超过 PID 参数中设定的上限(2000) */
    VAL_LIMIT(TempCtrl_PID.Output, -TempCtrl_PID.Param.LimitOutput, TempCtrl_PID.Param.LimitOutput);

    /* 向 PWM 加热通道写入功率值 */
    Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
