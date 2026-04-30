/**
  ******************************************************************************
  * @file           : vmc_kinematics.c
  * @brief          : VMC 运动学 — 把两条复杂的连杆腿变成两根简单的"棍子"
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   机器人的真实腿有2个关节: 大腿(Thigh)绕髋转,小腿(Calf)绕膝转。
  *   如果你直接基于这两个角度来做平衡控制,数学会极其复杂。
  *   所以这个文件做了三件事:
  *   1. 把"两个角度"通过正运动学公式变成"一根虚拟棍子"(长度L₀+倾角φ₀)
  *   2. 从电机力矩反馈反算出虚拟腿受的外力 F 和扭矩 Tp (逆动力学)
  *   3. 最后一步把算好的 F/Tp 通过雅可比矩阵"翻译"回4个关节电机的力矩
  *
  * 【核心职责】
  *   1. Joint_Angle_Offset: 电机角度 → 大腿/小腿摆角 (坐标系映射)
  *   2. VMC_Calculate:      正运动学 → 虚拟腿长 L₀ + 倾角 φ₀
  *   3. Measure_F_Tp:       逆动力学 → 虚拟力 F + 扭矩 Tp + 着地检测
  *   4. Joint_Tourgue:      雅可比 → 虚拟F/Tp → 4个真实关节力矩
  *
  * 【前置知识】
  *   - 连杆正运动学: θ → (x,y) 的几何转换
  *   - 雅可比矩阵: 虚拟力到关节力矩的"翻译器"
  *   - arm_sin_f32/arm_cos_f32: ARM CMSIS-DSP 库的高速三角函数
  *
  ******************************************************************************
  */

#include "vmc_kinematics.h"
#include "arm_math.h"

/**
 * VMC_Joint_Angle_Offset — 坐标系映射
 *
 * 🐣 电机汇报的"位置"和算法需要的"摆角"之间需要转换:
 *   - 左小腿: 电机角度 = 小腿摆角 (同方向)
 *   - 左大腿: 电机角度 + PI = 大腿摆角 (电机方向和腿方向差180°)
 *   - 右腿: 类似于左腿,但映射关系不同(因为右侧电机朝向镜像)
 */
void VMC_Joint_Angle_Offset(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* ====== LEFT LEG ====== */
    /* [左小腿摆角] (弧度 rad) | 电机0位置直接映射 */
    Control_Info->L_Leg_Info.Biased.Calf_Angle      = in->joint[0].position;

    /* [左大腿摆角] (弧度 rad) | PI + 电机1位置
       🐣 为什么加PI? 因为电机装在腿后面,转轴方向和腿摆角方向差了180° */
    Control_Info->L_Leg_Info.Biased.Thigh_Angle     = PI + in->joint[1].position;

    /* [大腿/小腿角速度] (弧度/秒 rad/s) | 直接映射 */
    Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot = in->joint[1].velocity;
    Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot  = in->joint[0].velocity;

    /* [大腿/小腿力矩反馈] (N·m) | 电机实际输出的扭矩
       🐣 用于逆动力学反算腿受的外力 */
    Control_Info->L_Leg_Info.Biased.T_Thigh         = in->joint[1].torque;
    Control_Info->L_Leg_Info.Biased.T_Calf          = in->joint[0].torque;

    /* ====== RIGHT LEG ====== */
    Control_Info->R_Leg_Info.Biased.Thigh_Angle     = in->joint[2].position;
    /* 🐣 右小腿: PI + 电机3位置 (同左腿映射) */
    Control_Info->R_Leg_Info.Biased.Calf_Angle      = PI + in->joint[3].position;
    Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot = in->joint[2].velocity;
    Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot  = in->joint[3].velocity;
    Control_Info->R_Leg_Info.Biased.T_Thigh         = in->joint[2].torque;
    Control_Info->R_Leg_Info.Biased.T_Calf          = in->joint[3].torque;
}

/**
 * VMC_Calculate — VMC 正运动学 (Virtual Model Control)
 *
 * 🐣 这是整个项目中最核心的数学步骤！
 *   目标: 把 2 个关节角度(θ₁=小腿, θ₂=大腿) 映射为 2 个简化参数(L₀=虚拟腿长, φ₀=虚拟摆角)
 *
 *   ============ 数学公式推导(不需要背) ============
 *   给定: θ₁=小腿摆角, θ₂=大腿摆角,
 *         a=AD(小腿连杆长), b=AH(大腿连杆长), K=AD/AH
 *
 *   中间变量:
 *     M = (θ₁ - θ₂)/2          // 夹角半差值
 *     N = (θ₁ + θ₂)/2          // 夹角半和值
 *     S = √(b² - a²·sin²(M))   // 勾股定理的推广
 *     t = a·cos(M) + S         // 脚底到髋关节的水平投影
 *     A = a·t·sin(M) / S       // 雅可比转换系数(力矩翻译器)
 *
 *   最终结果:
 *     φ₀ = N                   // 虚拟腿的摆角 = 两关节角度的平均值
 *     L₀ = t / K              // 虚拟腿长度 = 几何关系÷杆长比
 *
 *   🐣 通俗理解: "大腿和小腿各弯多少" → "从屁股到脚底,等效于一根多长的棍子、倾多少度"
 */
void VMC_Calculate(Control_Info_Typedef *Control_Info)
{
    /* ====== LEFT LEG 左腿 VMC 正运动学 ====== */

    /* 🐣 第1步: 取出机械参数,赋给局部变量 (不在结构体里占内存) */
    float L_a = Control_Info->L_Leg_Info.Biased.L_Calf_Link;   /* a = 小腿连杆长度 AD */
    float L_b = Control_Info->L_Leg_Info.Biased.L_Thigh_Link;  /* b = 大腿连杆长度 AH */

    /* M = -(小腿角 - 大腿角)/2
       🐣 负号是因为左右腿的关节角度定义方向不同 */
    float L_M = -(Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle) / 2.f;

    /* N = (小腿角 + 大腿角)/2
       🐣 两个角度的"均值方向",就是最终的虚拟腿方向 */
    float L_N = (Control_Info->L_Leg_Info.Biased.Calf_Angle + Control_Info->L_Leg_Info.Biased.Thigh_Angle) / 2.f;

    /* S_Radicand = b² - a²·sin²(M)
       🐣 这一步在算"在弯曲这么多时,连杆的长度投影还剩多少" */
    float L_S_Radicand = L_b * L_b - L_a * L_a * arm_sin_f32(L_M) * arm_sin_f32(L_M);

    float L_S;
    arm_sqrt_f32(L_S_Radicand, &L_S);
    /* 🔧 安全保护: 如果 S 接近零(浮点误差),强行设一个极小值,防止后面除零崩溃 */
    if (L_S < 1e-6f) L_S = 1e-6f;

    /* t = a·cos(M) + S
       🐣 这是"脚到髋关节的水平距离",也是虚拟腿长的分子 */
    float L_t = L_a * arm_cos_f32(L_M) + L_S;

    /* A = a·t·sin(M) / S
       🐣 这个 A 不是角度,是雅可比转换系数！
       后面 Joint_Tourgue_Calculate 中会用 A/K 把虚拟力 F 翻译成关节扭矩 */
    float L_A = (L_a * L_t * arm_sin_f32(L_M)) / L_S;

    /* 🐣 把局部计算结果写回结构体 */
    Control_Info->L_Leg_Info.A = L_A;                     /* 雅可比系数 */
    Control_Info->L_Leg_Info.Sip_Leg_Angle  = L_N;       /* φ₀ = N */
    Control_Info->L_Leg_Info.Sip_Leg_Length = L_t / Control_Info->L_Leg_Info.Biased.K;  /* L₀ = t/K */

    /* 🐣 J 点速度: X_J_Dot = (A×cosN×(ω₁-ω₂) - t×sinN×(ω₁+ω₂)) / (2·K)
       这是"轮子轴心的水平和垂直速度",用于传感器融合中的速度补偿 */
    Control_Info->L_Leg_Info.X_J_Dot = (L_A * arm_cos_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
                                      - L_t * arm_sin_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->L_Leg_Info.Biased.K);
    Control_Info->L_Leg_Info.Y_J_Dot = (L_A * arm_sin_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
                                      + L_t * arm_cos_f32(L_N) * (Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))
                                     / (2.f * Control_Info->L_Leg_Info.Biased.K);

    /* ====== RIGHT LEG 右腿 VMC 正运动学 (公式同上,变量独立) ====== */
    float R_a = Control_Info->R_Leg_Info.Biased.L_Calf_Link;
    float R_b = Control_Info->R_Leg_Info.Biased.L_Thigh_Link;
    /* 🐣 右腿的 M 公式中 Thigh 和 Calf 顺序跟左腿不一样 */
    float R_M = -(Control_Info->R_Leg_Info.Biased.Thigh_Angle - Control_Info->R_Leg_Info.Biased.Calf_Angle) / 2.f;
    float R_N = (Control_Info->R_Leg_Info.Biased.Calf_Angle + Control_Info->R_Leg_Info.Biased.Thigh_Angle) / 2.f;
    float R_S_Radicand = R_b * R_b - R_a * R_a * arm_sin_f32(R_M) * arm_sin_f32(R_M);
    float R_S;
    arm_sqrt_f32(R_S_Radicand, &R_S);
    if (R_S < 1e-6f) R_S = 1e-6f;
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

/**
 * VMC_Measure_F_Tp_Calculate — 逆动力学 + 着地检测
 *
 * 🐣 从电机力矩反馈反推虚拟腿受的外力,并判断腿是否接触地面。
 *
 *   公式:
 *     F  = K×(T_calf - T_thigh) / A     (虚拟推力)
 *     Tp = T_thigh + T_calf              (虚拟扭矩)
 *     FN = F×cos(Theta) + (Tp×sin(Theta))/L₀  (地面法向支持力)
 *
 *   离地判定: FN < 22N → Support.Flag = 1 (在空中)
 *   ⚠ 为什么是 22N？因为机器人自重约 200N(20kg),
 *   两条腿各承担约 100N。如果 FN < 22N,说明这条腿几乎没受力→离地了。
 */
void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info)
{
    /* ====== LEFT LEG ====== */
    /* F = K×(T_Calf - T_Thigh) / A
       🐣 这个公式的反直觉之处: 小腿力矩减大腿力矩,除以雅可比系数 = 虚拟推力
       因为小腿和大腿的力矩方向相反(一个伸一个收),差值才是净效果 */
    Control_Info->L_Leg_Info.Measure.F  = (Control_Info->L_Leg_Info.Biased.K * (Control_Info->L_Leg_Info.Biased.T_Calf - Control_Info->L_Leg_Info.Biased.T_Thigh)) / Control_Info->L_Leg_Info.A;
    /* Tp = T_Thigh + T_Calf
       🐣 扭矩 = 两个力矩的和,因为两个电机输出扭矩方向一致时叠加 */
    Control_Info->L_Leg_Info.Measure.Tp = (Control_Info->L_Leg_Info.Biased.T_Thigh + Control_Info->L_Leg_Info.Biased.T_Calf);

    /* ====== RIGHT LEG ====== */
    /* ⚠ 右腿公式: T_Thigh - T_Calf (跟左腿符号相反,因为左右腿坐标系镜像) */
    Control_Info->R_Leg_Info.Measure.F  = (Control_Info->R_Leg_Info.Biased.K * (Control_Info->R_Leg_Info.Biased.T_Thigh - Control_Info->R_Leg_Info.Biased.T_Calf)) / Control_Info->R_Leg_Info.A;
    Control_Info->R_Leg_Info.Measure.Tp = (Control_Info->R_Leg_Info.Biased.T_Thigh + Control_Info->R_Leg_Info.Biased.T_Calf);

    /* ====== 着地检测 ====== */
    if (Control_Info->Chassis_Situation == CHASSIS_BALANCE) {
        /* 仅在平衡状态下进行着地检测 */

        /* 地面支持力 FN = F×cos(Theta) + (Tp×sin(Theta))/L₀
           🐣 把虚拟腿的推力和扭矩投影到"垂直地面"方向,得到地面对轮子的反力 */
        Control_Info->L_Leg_Info.Support.FN = Control_Info->L_Leg_Info.Measure.F * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
                                            + ((Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))
                                               / Control_Info->L_Leg_Info.Sip_Leg_Length);

        /* 右腿同理,⚠ 注意有一个额外的负号(与左右腿坐标系有关) */
        Control_Info->R_Leg_Info.Support.FN = Control_Info->R_Leg_Info.Measure.F * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
                                            + (-(Control_Info->R_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))
                                               / Control_Info->R_Leg_Info.Sip_Leg_Length);

        /* 🐣 判断着地: FN < 22N → 腿在空中
           这个阈值是根据"半车重约100N,着地时FN应在50~150N之间"设定的 */
        Control_Info->L_Leg_Info.Support.Flag = (Control_Info->L_Leg_Info.Support.FN < 22.f);
        Control_Info->R_Leg_Info.Support.Flag = (Control_Info->R_Leg_Info.Support.FN < 22.f);
    } else {
        /* 非平衡状态下,着地检测无效,全部清零 */
        Control_Info->L_Leg_Info.Support.Flag = 0;
        Control_Info->R_Leg_Info.Support.Flag = 0;
        Control_Info->L_Leg_Info.Support.FN   = 100.f;
        Control_Info->R_Leg_Info.Support.FN   = 100.f;
    }
}

/**
 * VMC_Joint_Tourgue_Calculate — 雅可比矩阵转换 + 输出限幅
 *
 * 🐣 这是控制流水线的最后一步——把虚拟力F/扭矩Tp "翻译"回4个关节电机的力矩。
 *
 *   公式(左腿):
 *     T2(小腿) = (A/K) × F + Tp/2
 *     T1(大腿) = (-A/K) × F + Tp/2
 *
 *   为什么(A/K)前面有个负号？因为大腿和小腿的力矩方向相反——
 *   想给脚底一个向下的推力,大腿要"伸直"、小腿要"弯曲"。
 *
 *   ⚠ 限幅保护: 每个关节力矩被(-15, 15) N·m 夹住,防止烧电机。
 *   轮子电流也被(-10000, 10000) mA 夹住。
 */
void VMC_Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info)
{
    /* [力矩上限] (N·m) | 电机能安全输出的最大扭矩 */
    float Tourgue_max = 15.f;
    /* [电流上限] (mA) | 驱动器能安全输出的最大电流 */
    float Current_max = 10000;

    /* ====== LEFT LEG ====== */
    /* T2(小腿) = (A/K)×F + Tp/2
       🐣 左小腿=0号电机, T2对应小腿力矩 */
    Control_Info->L_Leg_Info.SendValue.T_Calf  = (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F) / Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp / 2.0f);
    /* T1(大腿) = (-A/K)×F + Tp/2
       🐣 左大腿=1号电机,注意前面有负号 */
    Control_Info->L_Leg_Info.SendValue.T_Thigh = (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F) / Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp / 2.0f);

    /* ====== RIGHT LEG ====== */
    /* ⚠ 右腿的映射关系和左腿略有不同:
       T2(大腿) = (A/K)×F + Tp/2  → 2号电机
       T1(小腿) = (-A/K)×F + Tp/2 → 3号电机 */
    Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F) / Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp / 2.0f);
    Control_Info->R_Leg_Info.SendValue.T_Calf  = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F) / Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp / 2.0f);

    /* ====== 轮子力矩 → 电流转换 ====== */
    /* 🐣 驱动轮使用"电流模式"控制,单位换算: 力矩(N·m) × 1200 ≈ 电流(mA)
       右轮取反: 因为左右轮镜像安装,同时正转方向相反 */
    Control_Info->L_Leg_Info.SendValue.Current = (int16_t)(Control_Info->L_Leg_Info.T * 1200.f);
    Control_Info->R_Leg_Info.SendValue.Current = (int16_t)(-Control_Info->R_Leg_Info.T * 1200.f);

    /* ====== ⚠ 限幅保护 (安全气囊!) ====== */
    /* 🐣 如果不加这些限幅,异常情况下可能输出 ±1000N·m 的力矩,
       电机会瞬间过流烧毁。限幅后保证在安全范围内。 */
    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.Current, -Current_max, Current_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.Current, -Current_max, Current_max);

    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Calf,  -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Thigh, -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Thigh, -Tourgue_max, Tourgue_max);
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Calf,  -Tourgue_max, Tourgue_max);
}
