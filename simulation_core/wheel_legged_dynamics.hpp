#pragma once
/**
 * @file    wheel_legged_dynamics.hpp
 * @brief   Sim-Core: 偏置并联轮腿机器人纯算法 C++ 封装
 *
 * 提取自 STM32 的 vmc_kinematics.c + lqr_controller.c，
 * 剥离全部 HAL/FreeRTOS 依赖，只保留核心数学。
 *
 * 输入:  6维状态向量 [θ, dθ, x, dx, φ, dφ] + 4个关节角度
 * 输出:  6维力矩向量 [T_calf_L, T_thigh_L, T_thigh_R, T_calf_R, I_wheel_L, I_wheel_R]
 */

#include <array>
#include <vector>
#include <cmath>

namespace sim_core {

// ========== 机械参数 ==========
struct RobotParams {
    // 连杆
    double thigh_link_len = 0.118114;   // 大腿连杆 AH (m)
    double calf_link_len  = 0.100;     // 小腿连杆 AD (m)
    double link_ratio_K   = 0.465116;  // K = AD/AH

    // 轮子
    double wheel_radius   = 0.055;     // (m)
    double wheel_reduction= 15.0;      // 减速比

    // 限幅
    double joint_torque_max   = 15.0;  // N·m
    double wheel_current_max  = 10000; // mA
    double torque_to_current  = 1200;  // N·m → mA 系数

    // 离地检测
    double support_fn_threshold = 22.0; // N

    // 液压方向
    double pi = M_PI;
};

// ========== LQR 增益调度的 12 组多项式系数 ==========
// Kij[0]=占位, Kij[1]=c1(L³), Kij[2]=c2(L²), Kij[3]=c3(L), Kij[4]=c4
constexpr std::array<std::array<double, 5>, 12> K_COEFFS = {{
    // K11~K16 (第0行: 轮子力矩 T)
    {{0, -344.130023,  397.724995, -265.059481,  -4.941964}},
    {{0,   11.842778,  -18.891159,  -27.922778,   0.234829}},
    {{0, -288.953787,  281.637253,  -94.596365, -10.720163}},
    {{0, -177.996259,  181.622915,  -75.159282,  -7.728459}},
    {{0, -835.889683,  930.198548, -389.150660,  74.543061}},
    {{0,  -58.542501,   66.926377,  -29.456008,   6.433743}},
    // K21~K26 (第1行: 关节扭矩 Tp)
    {{0,  178.165050, -120.123702,  -0.177096,  29.334646}},
    {{0,   38.945329,  -38.984286,  14.882355,   2.371578}},
    {{0, -527.320926,  586.755646,-245.391894,  46.899771}},
    {{0, -343.006363,  380.367381,-159.679912,  32.616099}},
    {{0, 1840.588017,-1794.197881, 602.816532,  67.365929}},
    {{0,  151.012003, -149.438347,  51.534782,   1.364543}},
}};

// ========== 正运动学输出 ==========
struct VmcResult {
    double L0;        // 虚拟腿长 (m)
    double phi0;      // 虚拟腿倾角 (rad)
    double A_coeff;   // 雅可比转换系数
    double X_J_dot;   // J点X方向速度
    double Y_J_dot;   // J点Y方向速度
};

// ========== LQR 输出 ==========
struct LqrResult {
    double balance_T;   // 轮子平衡力矩 (N·m)
    double balance_Tp;  // 关节平衡扭矩 (N·m)
    std::array<double, 6> K_row0;  // 第0行增益
    std::array<double, 6> K_row1;  // 第1行增益
};

// ========== 电机命令输出 ==========
struct MotorCommand {
    std::array<double, 4> joint_torque;   // [左小腿, 左大腿, 右大腿, 右小腿] N·m
    std::array<double, 2> wheel_current;  // [左轮, 右轮] mA
};

// ========== 完整状态 ==========
struct RobotState {
    std::array<double, 4> joint_angles;    // 4个关节角度 (rad)
    std::array<double, 4> joint_velocities;// 4个关节角速度 (rad/s)
    std::array<double, 6> state_vector;    // LQR 6维状态 [θ,dθ,x,dx,φ,dφ]
    double chassis_velocity = 0.0;         // 底盘速度 (m/s)
    double target_velocity = 0.0;          // 目标速度 (m/s)
};

class WheelLeggedDynamics {
public:
    WheelLeggedDynamics();
    explicit WheelLeggedDynamics(const RobotParams& params);

    /**
     * @brief  核心入口: 给定机器人完整状态，输出6个电机的目标指令
     * @param  state 机器人当前状态 (关节角度/速度 + 6维LQR状态)
     * @return MotorCommand 4个关节力矩 + 2个轮子电流
     */
    MotorCommand compute_torques(const RobotState& state);

    /**
     * @brief  正运动学: 2个关节角度 → 虚拟腿长 L₀ + 倾角 φ₀
     * @param  calf_angle   小腿摆角 (rad)
     * @param  thigh_angle  大腿摆角 (rad)
     * @param  is_left_leg  true=左腿, false=右腿
     * @return VmcResult
     */
    VmcResult vmc_forward_kinematics(double calf_angle, double thigh_angle, bool is_left_leg) const;

    /**
     * @brief  逆动力学: 关节力矩反馈 → 虚拟推力 F + 扭矩 Tp
     * @param  T_calf      小腿电机力矩 (N·m)
     * @param  T_thigh     大腿电机力矩 (N·m)
     * @param  A_coeff     雅可比转换系数
     * @param  link_ratio  连杆长度比 K
     * @param  is_left_leg true=左腿
     * @return pair{F, Tp}
     */
    std::pair<double, double> inverse_dynamics(double T_calf, double T_thigh,
                                                double A_coeff, double link_ratio,
                                                bool is_left_leg) const;

    /**
     * @brief  着地检测: 计算地面支持力 FN
     * @param  F    虚拟推力 (N)
     * @param  Tp   虚拟扭矩 (N·m)
     * @param  Theta 虚拟腿倾角 (rad)
     * @param  L0   虚拟腿长 (m)
     * @return pair{FN, is_supported}
     */
    std::pair<double, bool> contact_detection(double F, double Tp,
                                               double Theta, double L0) const;

    /**
     * @brief  LQR 增益调度: K = f(L₀)
     * @param  L0  虚拟腿长 (m)
     * @return LqrResult (2×6 增益矩阵 + 平衡力矩/扭矩)
     */
    LqrResult lqr_gain_schedule(double L0, const std::array<double, 6>& X) const;

    /**
     * @brief  雅可比逆矩阵: 虚拟力F/扭矩Tp → 4个真实关节力矩
     * @param  F          虚拟推力 (N)
     * @param  Tp         虚拟扭矩 (N·m)
     * @param  A_coeff    雅可比系数
     * @param  link_ratio 连杆比 K
     * @param  is_left_leg true=左腿
     * @return pair{T_calf, T_thigh}
     */
    std::pair<double, double> jacobian_torque_mapping(double F, double Tp,
                                                       double A_coeff, double link_ratio,
                                                       bool is_left_leg) const;

    // 参数 getter/setter
    const RobotParams& params() const { return params_; }
    void set_params(const RobotParams& p) { params_ = p; }

private:
    RobotParams params_;
    static double clamp(double v, double lo, double hi);
};

/**
 * @brief  快速多项式求值 (霍纳法则)
 * @param  coeffs [c4, c3, c2, c1, c0] 即 y = c0·x⁴ + c1·x³ + c2·x² + c3·x + c4
 *                 但本项目用 Kij[1]·L³ + Kij[2]·L² + Kij[3]·L + Kij[4]
 * @param  x      自变量 (腿长 L₀)
 * @return double 多项式值
 */
inline double polyval_horner(const std::array<double, 5>& coeffs, double x) {
    return ((coeffs[1] * x + coeffs[2]) * x + coeffs[3]) * x + coeffs[4];
}

} // namespace sim_core
