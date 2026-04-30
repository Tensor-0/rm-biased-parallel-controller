/**
  ******************************************************************************
  * @file           : chassis_control.c
  * @brief          : 底盘高层控制 — 前进/转向/高度/横滚/腿长
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   这是机器人的"方向盘+刹车+油门+悬挂",负责把遥控器的操作
  *   翻译成机器人的实际动作。四个独立的控制通道:
  *   1. 前进后退: 摇杆→速度目标,用斜坡函数平滑过渡
  *   2. 左右转向: 摇杆→偏航角度差,用串级PID控制
  *   3. 高度切换: 开关→腿长目标,用斜坡函数平滑切换
  *   4. 横滚补偿: IMU Roll角→左右腿长差+推力差,维持水平
  *   5. 腿长控制: 总目标腿长→PID→推力+重力补偿
  *
  * 【核心职责】
  *   1. Move_Control:    前进后退 + 偏航转向
  *   2. Height_Control:  高/低底盘切换
  *   3. Roll_Control:    横滚补偿 (腿长差+推力差)
  *   4. Leg_Length:      腿长 PID + 重力补偿
  *
  * 【前置知识】
  *   - f_Ramp_Calc(x, target, rate): 斜坡函数,让 x 平滑地趋向 target
  *     rate 越大越"急促",越小越"慵懒"
  *   - VAL_LIMIT(x, min, max): 限幅宏,把 x 夹在 [min, max] 之间
  *
  ******************************************************************************
  */

#include "chassis_control.h"
#include "PID.h"
#include "Ramp.h"
#include "arm_math.h"

extern PID_Info_TypeDef PID_Leg_length_F[2];
extern PID_Info_TypeDef PID_Leg_Roll_F;
extern PID_Info_TypeDef PID_Yaw[2];

/**
 * Chassis_Move_Control — 前进后退 + 偏航转向
 *
 * 🐣 遥控器 ch[2] 和 ch[3] 控制移动:
 *   ch[3]: 前进/后退摇杆 (范围: [-660, 660])
 *   ch[2]: 转向摇杆 (范围: [-660, 660])
 *
 *   前进后退: 斜坡函数逐步改变目标速度,防止突然加速摔倒。
 *   转向: 串级PID——外环把"角度差"转成"目标角速度",内环把"目标角速度"转成力矩。
 */
void Chassis_Move_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* [前进响应快慢] | 越小越慢/越平滑 | 0.001 = 非常缓慢的起步 */
    float K_Velocity = 0.001f;
    /* [刹车响应快慢] | 比起步稍快,确保能快速停下 */
    float K_Brake    = 0.002f;
    /* [转向响应] | 0.2 适中,太快会晃,太慢转不动 */
    float K_Yaw_P    = 0.2f;

    /* ====== 前进/后退 ======
       🐣 当摇杆不在中心(ch[3]≠0)时:
       - 用斜坡函数(ramp)修改目标速度: 当前值 → 摇杆值×0.00242
       - 同时把底盘位置实时清零 → "我在移动,位置环不工作,只控速度"
       当摇杆归零: 用斜坡函数"溜车"(慢慢归零),实现平滑刹车 */
    if (in->rc.ch[3] != 0) {
        /* ch[3]范围[-660,660], 乘以0.00242 映射到约[-1.6, 1.6] m/s 的速度范围 */
        Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity, -in->rc.ch[3] * 0.00242f, K_Velocity);
        Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
    } else {
        Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity, 0, K_Brake);
    }
    /* ⚠ 安全限速: 不管摇杆推多大,底盘速度≤1.6m/s (约5.8km/h) */
    VAL_LIMIT(Control_Info->Target_Velocity, -1.6f, 1.6f);

    /* ====== 偏航(左右转) ====== */
    /* ch[2]的值映射到目标偏航角度(度),用斜坡函数平滑变化 */
    Control_Info->Yaw_Err = f_Ramp_Calc(Control_Info->Yaw_Err, (in->rc.ch[2] * RemoteToDegrees), K_Yaw_P);

    /* 🐣 角度归一化: 把 [-360°, 360°] 映射到 [-180°, 180°]
       因为"转 350°"等价于"反方向转10°",取近不取远 */
    if (Control_Info->Yaw_Err >= 180.f)
        Control_Info->Yaw_Err -= 360.f;
    else if (Control_Info->Yaw_Err <= -180.f)
        Control_Info->Yaw_Err += 360.f;

    /* 🐣 串级PID: 外环(位置)输出作为内环(速度)的目标
       PID_Yaw[0]: "我要转X度" → 算出"我该转多快"
       PID_Yaw[1]: "我该转Y°/s" → 算出"我该用力矩Z" */
    PID_Calculate(&PID_Yaw[0], 0, Control_Info->Yaw_Err);
    PID_Calculate(&PID_Yaw[1], PID_Yaw[0].Output, in->ins.Yaw_Gyro);

    /* 🐣 左右腿加相反方向的转向力矩 = 差速转向
       左腿 +Turn_T, 右腿 -Turn_T → 左轮加速右轮减速 → 右转 */
    Control_Info->L_Leg_Info.Moment.Turn_T =  PID_Yaw[1].Output;
    Control_Info->R_Leg_Info.Moment.Turn_T = -PID_Yaw[1].Output;
}

/**
 * Chassis_Height_Control — 高/低底盘切换
 *
 * 🐣 遥控器开关 s[1]==1 → 高腿长(越野模式), s[1]!=1 → 低腿长(公路模式)
 *   K_Height = 0.0003 = 非常慢的切换速度,确保腿长变化不会导致机器摔倒
 */
void Chassis_Height_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    float K_Height = 0.00030f;
    if (in->rc.s[1] == 1) {
        /* 高腿长: 缓慢增加到 Control_Info.Base_Leg_Length_High (0.20m) */
        Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_High, K_Height);
        Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_High, K_Height);
    } else {
        /* 低腿长: 缓慢降低到 Control_Info.Base_Leg_Length_Low (0.14m) */
        Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_Low, K_Height);
        Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Base_Leg_Length, Control_Info->Base_Leg_Length_Low, K_Height);
    }
}

/**
 * Chassis_Roll_Control — 横滚补偿
 *
 * 🐣 当机器人站在斜坡上,或一脚踩到高物体时,机身会左右倾斜(Roll)。
 *   横滚补偿做两件事:
 *   1. 几何补偿: 算出一条腿要多长才能把机身"撑平"
 *   2. 力补偿: 用PID给"被压着的那条腿"更多推力
 */
void Chassis_Roll_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* [实际两腿长度差] (米 m) | 正值=左腿比右腿长 */
    Control_Info->Roll.Length_Diff = Control_Info->L_Leg_Info.Sip_Leg_Length - Control_Info->R_Leg_Info.Sip_Leg_Length;

    /* [当前横滚角] (弧度 rad) | 一阶滤波: IMU 测量值 → 平滑后的角度
       🐣 -0.0291rad 是 IMU 安装时的"零点偏移",扣掉才能得到真实的机身水平角 */
    Control_Info->Roll.Angle = f_Ramp_Calc(Control_Info->Roll.Angle, (-in->ins.Angle[1] - 0.0291f), 0.003f);

    /* [腿长差产生的横滚角正切值]
       tan(Δθ) = 腿长差 ÷ 轮间距 */
    Control_Info->Roll.Tan_Length_Diff_Angle = Control_Info->Roll.Length_Diff / Control_Info->Roll.Distance_Two_Wheel;

    /* [机身横滚角的正切值]
       tan(θ) = sin(θ)/cos(θ) */
    Control_Info->Roll.Tan_Angle = arm_sin_f32(Control_Info->Roll.Angle) / arm_cos_f32(Control_Info->Roll.Angle);

    /* [实际坡度角的正切值]
       🐣 两角和的正切公式: tan(α+β) = (tanα+tanβ)/(1-tanα·tanβ)
       α = 机身横滚角, β = 腿长差横滚角
       需要这个公式是因为"腿长差实际上是让两轮子落在不同高度,等价于机身站在斜坡上" */
    Control_Info->Roll.Tan_Slope_Angle = (Control_Info->Roll.Tan_Angle + Control_Info->Roll.Tan_Length_Diff_Angle)
                                       / (1.0f - (Control_Info->Roll.Tan_Angle * Control_Info->Roll.Tan_Length_Diff_Angle));

    /* [横滚补偿腿长] (米 m)
       🐣 坡度角×轮间距÷2 = 需要哪条腿多长才能把机身"撑平"
       左腿多伸长,右腿多缩短 > 这样机身就水平了 */
    Control_Info->L_Leg_Info.Roll_Leg_Length =  (Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel) / 2.0f;
    Control_Info->R_Leg_Info.Roll_Leg_Length = -(Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel) / 2.0f;

    /* [横滚推力补偿]
       🐣 PID 根据横滚角计算补偿力,左腿"往下压"右腿"往上顶",恢复水平
       PID_Leg_Roll_F: Kp=50, Kd=25 → 快速但不过冲 */
    Control_Info->L_Leg_Info.Moment.Roll_F = -PID_Calculate(&PID_Leg_Roll_F, 0.f, Control_Info->Roll.Angle);
    Control_Info->R_Leg_Info.Moment.Roll_F =  PID_Calculate(&PID_Leg_Roll_F, 0.f, Control_Info->Roll.Angle);
}

/**
 * Leg_Length_Control — 腿长控制 + 综合推力
 *
 * 🐣 把基础腿长(Base) + 横滚补偿(Roll) = 总目标腿长(Total)
 *   然后用 PID 控制实际腿长趋近目标,同时加上重力补偿(抵消自重)。
 */
void Leg_Length_Control(Control_Info_Typedef *Control_Info)
{
    float K_Length = 0.35f;

    /* 总目标腿长 = 基础腿长 + 横滚补偿,用斜坡函数平滑变化 */
    Control_Info->L_Leg_Info.Total_Leg_Length = f_Ramp_Calc(Control_Info->L_Leg_Info.Total_Leg_Length,
        Control_Info->L_Leg_Info.Base_Leg_Length + Control_Info->L_Leg_Info.Roll_Leg_Length, K_Length);
    Control_Info->R_Leg_Info.Total_Leg_Length = f_Ramp_Calc(Control_Info->R_Leg_Info.Total_Leg_Length,
        Control_Info->R_Leg_Info.Base_Leg_Length + Control_Info->R_Leg_Info.Roll_Leg_Length, K_Length);

    /* ⚠ 限幅: 腿长不能超过机械结构允许的范围 [0.14, 0.32] 米 */
    VAL_LIMIT(Control_Info->L_Leg_Info.Total_Leg_Length, 0.14f, 0.32f);
    VAL_LIMIT(Control_Info->R_Leg_Info.Total_Leg_Length, 0.14f, 0.32f);

    /* PID 控制: 误差 = 目标腿长 - 实际虚拟腿长 → 输出推力
       🐣 PID_Leg_length_F: Kp=1300 很高,因为腿长1mm误差都要大力推回去 */
    Control_Info->L_Leg_Info.Moment.Leg_Length_F = PID_Calculate(&PID_Leg_length_F[0], Control_Info->L_Leg_Info.Total_Leg_Length, Control_Info->L_Leg_Info.Sip_Leg_Length);
    Control_Info->R_Leg_Info.Moment.Leg_Length_F = PID_Calculate(&PID_Leg_length_F[1], Control_Info->R_Leg_Info.Total_Leg_Length, Control_Info->R_Leg_Info.Sip_Leg_Length);

    /* [重力补偿] (牛顿 N) | 100N ≈ 半车重
       🐣 这个值让腿始终有一个向下的基线推力,抵消机器人自重 */
    Control_Info->R_Leg_Info.Gravity_Compensation = 100.f;
    Control_Info->L_Leg_Info.Gravity_Compensation = 100.f;

    /* ⚠ 支撑腿特殊处理:
       - 支撑腿不需要横滚补偿力(它本来就在撑地)
       - 保持基础重力补偿即可 */
    if (Control_Info->L_Leg_Info.Support.Flag == 1) {
        Control_Info->L_Leg_Info.Moment.Roll_F = 0;
        Control_Info->L_Leg_Info.Gravity_Compensation = 100.f;
    }
    if (Control_Info->R_Leg_Info.Support.Flag == 1) {
        Control_Info->R_Leg_Info.Moment.Roll_F = 0;
        Control_Info->R_Leg_Info.Gravity_Compensation = 100.f;
    }

    /* 🐣 综合推力 F = 腿长控制力 + 横滚补偿力 + 重力补偿
       这三个力都在"垂直地面"的方向上,共同决定腿的最终推力 */
    Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F + Control_Info->L_Leg_Info.Moment.Roll_F + Control_Info->L_Leg_Info.Gravity_Compensation;
    Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F + Control_Info->R_Leg_Info.Moment.Roll_F + Control_Info->R_Leg_Info.Gravity_Compensation;
}
