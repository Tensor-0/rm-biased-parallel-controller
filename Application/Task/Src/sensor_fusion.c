/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @brief          : Sensor fusion: 1st-order lag filter (α=0.8) blending wheel odometry and IMU-derived acceleration.
  *
  * ======  新手必读 ======
  *
  * 【这个文件在干什么？】
  *   机器人有两个信息来源: IMU(惯性传感器,给你倾角/角速度/加速度)和
  *   轮速计(电机转速,给你底盘速度)。但两者都不完美:
  *   - IMU 加速度有噪声,直接积分两次会漂到天上去
  *   - 轮速计有打滑(比如轮子空转时,读数是假的)
  *   所以这个文件做的事情就是"两个人说速度是多少,我取个加权平均"。
  *
  * 【核心职责】
  *   1. 从 IMU 读取倾角 Phi 和角速度 Phi_dot (左右腿各一份)
  *   2. 从轮速计读取 RPM,转换成线速度,减去机身自转+腿部摆动
  *   3. "一阶滞后滤波": 0.8×预测值 + 0.2×测量值 = 融合速度
  *   4. 计算底盘加速度 Accel (用于速度预测)
  *   5. CHASSIS_WEAK 状态下清零所有状态量(防止错误控制)
  *
  * 【前置知识】
  *   - 互补滤波的思想: 相信低频的轮速计(长期准),相信高频的IMU(短期快)
  *   - 融合公式: Fusion = α×Predict + (1-α)×Measure, α=0.8
  *     α 越大,越"懒",响应慢但平滑; α 越小,越"敏感",响应快但噪声大
  *
  ******************************************************************************
  */

#include "sensor_fusion.h"
#include "arm_math.h"

/* [重力加速度] (m/s²) | 地球重力常数,用于修正 IMU 加速度读数
    IMU 测到的加速度 = 真实运动加速度 + 重力分量。要扣除重力才知道"真速度"。 */
#define GravityAccel 9.81f

void SensorFusion_Measure_Update(Control_Info_Typedef *Control_Info,
                                  const control_input_snapshot_t *in)
{
    /* ====== 共享 IMU 数据提取 ======
        Phi 是"机身倾角"——机器人前后倒了多少度。
       左右腿站在同一块机身上,所以 Phi 和 Phi_dot 对两条腿是一样的。
       这里提到外面,避免重复读两次 IMU。 */
    /* [机身倾角] (弧度 rad) | 范围: [-π/6, π/6] | 正值=向后仰
        = -IMU测的Pitch角 - 安装补偿角。负号是因为IMU方向跟控制坐标系可能反了 */
    float phi_shared     = -in->ins.Angle[2] - Control_Info->L_Leg_Info.Phi_Comp_Angle;
    /* [机身倾角速度] (弧度/秒 rad/s) | 范围: [-5, 5] | 正值=正在向后倒 */
    float phi_dot_shared = -in->ins.Gyro[0];

    /* ====== LEFT LEG 左腿姿态 ====== */

    Control_Info->L_Leg_Info.Measure.Phi     = phi_shared;
    Control_Info->L_Leg_Info.Measure.Phi_dot = phi_dot_shared;

    /* [虚拟腿倾角 Theta] (弧度 rad) | 范围: [-π/4, π/4]
        Theta = (地平线方向) - 虚拟腿角度 - 机身倾斜
       = (PI/2 - Sip_Leg_Angle) - Phi
       这就是 LQR 状态向量 X[0] 的测量值——"虚拟腿偏离竖直方向多少"。 */
    Control_Info->L_Leg_Info.Measure.Theta   = ((PI/2) - Control_Info->L_Leg_Info.Sip_Leg_Angle) - Control_Info->L_Leg_Info.Measure.Phi;

    /* [虚拟腿倾角速度] (弧度/秒 rad/s)
        把 J 点的 X/Y 方向线速度,投影到"垂直于虚拟腿"的方向上,
       再除以虚拟腿长,得到角速度。这叫"坐标投影+归一化"。 */
    Control_Info->L_Leg_Info.Measure.Theta_dot = (Control_Info->L_Leg_Info.X_J_Dot * arm_cos_f32(-Control_Info->L_Leg_Info.Measure.Theta)
                                                + Control_Info->L_Leg_Info.Y_J_Dot * arm_sin_f32(-Control_Info->L_Leg_Info.Measure.Theta))
                                               / Control_Info->L_Leg_Info.Sip_Leg_Length;

    /* ====== RIGHT LEG 右腿姿态 ====== */
    Control_Info->R_Leg_Info.Measure.Phi     = phi_shared;
    Control_Info->R_Leg_Info.Measure.Phi_dot = phi_dot_shared;

    /*  右腿的虚拟腿倾角公式跟左腿符号略有不同,因为坐标系定义差异 */
    Control_Info->R_Leg_Info.Measure.Theta   = (Control_Info->R_Leg_Info.Sip_Leg_Angle - (PI/2)) - Control_Info->R_Leg_Info.Measure.Phi;

    Control_Info->R_Leg_Info.Measure.Theta_dot = (-Control_Info->R_Leg_Info.X_J_Dot * arm_cos_f32(-Control_Info->R_Leg_Info.Measure.Theta)
                                                 + Control_Info->R_Leg_Info.Y_J_Dot * arm_sin_f32(-Control_Info->R_Leg_Info.Measure.Theta))
                                                / Control_Info->R_Leg_Info.Sip_Leg_Length;

    /* ====== LEFT LEG 左腿轮速融合 ====== */

    /*  第1步: RPM → 角速度 (弧度/秒)
       RPM×(π/30) = 弧度/秒, 再÷15 = 减速比(电机转15圈轮子转1圈) */
    Control_Info->L_Leg_Info.Velocity.Wheel = in->wheel[0].velocity * (PI / 30.f) / 15.f;

    /*  第2步: 补偿后的真实轮速
       W = 轮子转速 - 机身旋转速度 + 腿部摆动速度
       "如果轮子在转,但转是因为机身整体在转,那不算真实移动" */
    Control_Info->L_Leg_Info.Velocity.W     = (Control_Info->L_Leg_Info.Velocity.Wheel
                                              - Control_Info->L_Leg_Info.Measure.Phi_dot
                                              + Control_Info->L_Leg_Info.Measure.Theta_dot);

    /*  第3步: 角速度 → 线速度
       X = W × r, 轮半径 0.055m, 单位: m/s */
    Control_Info->L_Leg_Info.Velocity.X     = Control_Info->L_Leg_Info.Velocity.W * 0.055f;

    /*  第4步: 腿长变化率
       只取 Y_J_Dot 在 Theta 方向上的投影,因为"腿长变化"只跟垂直方向有关 */
    Control_Info->L_Leg_Info.Sip_Leg_Length_dot = Control_Info->L_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta);

    /*  第5步: Body = 原始线速度(基础版,不含运动学高级补偿) */
    Control_Info->L_Leg_Info.Velocity.Body  = Control_Info->L_Leg_Info.Velocity.X;

    /*  第6步: 预测速度 = 上一周期的融合速度 + 加速度×Δt
       Δt = 0.001s (1ms周期), Accel 来自 IMU 加速度(扣除重力后)
       "如果我知道上一瞬间跑多快,也知道加速度,就能猜出现在应该跑多快" */
    Control_Info->L_Leg_Info.Predict_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion + Control_Info->Accel * 0.001f;

    /*  第7步: 融合速度 (核心公式!)
       Fusion = 0.8 × Predict + 0.2 × Body
       - 0.8 权重给"我觉得",相信自己的预测(平滑,抗噪声)
       - 0.2 权重给"我看到",相信测量(修正累积误差)
       这就是一阶滞后滤波器 —— 机器人控制中最常见的数据融合方式 */
    Control_Info->L_Leg_Info.Velocity.Fusion  = Control_Info->L_Leg_Info.Velocity.Predict * 0.8f + Control_Info->L_Leg_Info.Velocity.Body * 0.2f;

    /* 最终: 融合速度 → 赋值给底盘速度测量值 (LQR 状态向量 X[3] 的测量值) */
    Control_Info->L_Leg_Info.Measure.Chassis_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion;

    /* ====== RIGHT LEG 右腿轮速融合 (原理同上,注意右轮极性取反) ====== */
    /* ⚠ 右轮 RPM 取反: 因为左右轮镜像安装,同方向旋转时 RPM 符号相反 */
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

    /* ====== 底盘总体速度与位置 ====== */

    /* 综合底盘速度 = 左右腿速度的平均值
        正常时两腿速度应该差不多,取平均能减少测量误差 */
    Control_Info->Chassis_Velocity = (Control_Info->L_Leg_Info.Measure.Chassis_Velocity + Control_Info->R_Leg_Info.Measure.Chassis_Velocity) / 2.f;

    /*  位置更新逻辑:
       - 如果目标速度为0(停止/定速模式) → 积分速度得到位置
       - 如果目标速度非0(正在移动)   → 位置归零(位置环不工作,只控速度)
       为什么？因为"我要走到正前方2米"这个功能还在开发,目前位置只用来自平衡 */
    if (Control_Info->Target_Velocity == 0) {
        /* 位置 = 位置 + 速度 × Δt (Δt = 0.001s) */
        Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity * 0.001f;
        Control_Info->R_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity * 0.001f;
    } else {
        Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
    }

    /* ====== 底盘加速度计算 ======
        基本物理公式: IMU测的加速度 = 运动加速度 + 重力分量 + 向心加速度
       我们要的是"运动加速度",所以要扣除重力和向心项:

       Accel = [(-a_y + ω_z²×r - g×sinΦ)×cosΦ] + [(a_z - g×cosΦ)×sinΦ]

       其中:
       - -a_y: Y轴加速度 (前后方向)
       - ω_z²×0.155: 向心加速度 (0.155m 是 IMU 到旋转中心的距离)
       - g×sinΦ: 重力在前后方向的分量 (倾斜时重力会"拽着你往前滑")
       - a_z - g×cosΦ: Z轴加速度扣除重力后有效分量
       - 乘以 cosΦ/sinΦ: 投影到水平方向 */
    Control_Info->Accel = (float)((-in->ins.Accel[1] + powf(in->ins.Gyro[2], 2) * 0.155f)
                          - GravityAccel * arm_sin_f32(-in->ins.Angle[2])) * arm_cos_f32(-in->ins.Angle[2])
                        + (in->ins.Accel[2] - GravityAccel * arm_cos_f32(-in->ins.Angle[2])) * arm_sin_f32(-in->ins.Angle[2]);

    /* ====== CHASSIS_WEAK 状态安全保护 ======
        如果底盘处于"虚弱"状态(没初始化好/关机了),所有状态量强行清零。
       如果不这样做,残存的错误状态会通过 LQR 产生错误输出,
       可能在下次启动时导致机器人突然猛烈动作——这是非常危险的！
       所以这个 if 块不是可选的,而是"安全气囊"。 */
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
