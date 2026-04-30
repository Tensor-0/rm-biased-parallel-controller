#include "wheel_legged_dynamics.hpp"
#include <algorithm>
#include <stdexcept>

namespace sim_core {

// ─── 构造 ────────────────────────────────────────
WheelLeggedDynamics::WheelLeggedDynamics() = default;
WheelLeggedDynamics::WheelLeggedDynamics(const RobotParams& params)
    : params_(params) {}

// ─── 工具函数 ────────────────────────────────────
double WheelLeggedDynamics::clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// ─── 正运动学 ────────────────────────────────────
VmcResult WheelLeggedDynamics::vmc_forward_kinematics(
        double calf_angle, double thigh_angle, bool is_left_leg) const {
    const double a = params_.calf_link_len;
    const double b = params_.thigh_link_len;
    const double M = -(calf_angle - thigh_angle) / 2.0;
    const double N = (calf_angle + thigh_angle) / 2.0;
    const double S_radicand = b*b - a*a * std::sin(M)*std::sin(M);
    double S = std::sqrt(std::max(0.0, S_radicand));
    if (S < 1e-6) S = 1e-6;   // 防除零
    const double t = a * std::cos(M) + S;
    const double A = (a * t * std::sin(M)) / S;

    VmcResult r;
    r.A_coeff = A;
    r.phi0    = N;
    r.L0      = t / params_.link_ratio_K;

    // J点速度 (如果需要)
    // 这里暂不计算因为没有关节角速度输入 — 仿真直接提供状态向量
    r.X_J_dot = 0.0;
    r.Y_J_dot = 0.0;
    return r;
}

// ─── 逆动力学 ────────────────────────────────────
std::pair<double, double> WheelLeggedDynamics::inverse_dynamics(
        double T_calf, double T_thigh, double A_coeff, double link_ratio,
        bool is_left_leg) const {
    if (is_left_leg) {
        double F  = (link_ratio * (T_calf - T_thigh)) / A_coeff;
        double Tp = T_thigh + T_calf;
        return {F, Tp};
    } else {
        double F  = (link_ratio * (T_thigh - T_calf)) / A_coeff;
        double Tp = T_thigh + T_calf;
        return {F, Tp};
    }
}

// ─── 着地检测 ────────────────────────────────────
std::pair<double, bool> WheelLeggedDynamics::contact_detection(
        double F, double Tp, double Theta, double L0) const {
    double FN = F * std::cos(Theta) + (Tp * std::sin(Theta)) / L0;
    bool supported = (FN >= params_.support_fn_threshold);
    return {FN, supported};
}

// ─── LQR 增益调度 ────────────────────────────────
LqrResult WheelLeggedDynamics::lqr_gain_schedule(
        double L0, const std::array<double, 6>& X) const {
    LqrResult r{};
    r.balance_T  = 0.0;
    r.balance_Tp = 0.0;

    // 计算 K 矩阵 (2×6) via 霍纳法则
    for (int j = 0; j < 6; ++j) {
        r.K_row0[j] = polyval_horner(K_COEFFS[j],   L0);  // K11~K16
        r.K_row1[j] = polyval_horner(K_COEFFS[j+6], L0);  // K21~K26
    }

    // u = K·X
    for (int j = 0; j < 6; ++j) {
        r.balance_T  += r.K_row0[j] * X[j];
        r.balance_Tp += r.K_row1[j] * X[j];
    }
    // 左腿符号取反 (与原 STM32 一致)
    // 注意: 这里左右腿共用同一个 K, 实际使用时区分
    return r;
}

// ─── 雅可比逆矩阵 ────────────────────────────────
std::pair<double, double> WheelLeggedDynamics::jacobian_torque_mapping(
        double F, double Tp, double A_coeff, double link_ratio,
        bool is_left_leg) const {
    double AK = A_coeff / link_ratio;
    if (is_left_leg) {
        // T_calf = (A/K)*F + Tp/2
        // T_thigh = (-A/K)*F + Tp/2
        double T_calf  =  AK * F + Tp / 2.0;
        double T_thigh = -AK * F + Tp / 2.0;
        return {T_calf, T_thigh};
    } else {
        // 右腿: T_thigh = (A/K)*F + Tp/2, T_calf = (-A/K)*F + Tp/2
        double T_thigh =  AK * F + Tp / 2.0;
        double T_calf  = -AK * F + Tp / 2.0;
        return {T_calf, T_thigh};
    }
}

// ─── 核心入口 ────────────────────────────────────
MotorCommand WheelLeggedDynamics::compute_torques(const RobotState& state) {
    MotorCommand cmd{};

    // ────── 1. 左右腿 VMC 正运动学 ──────
    //  从 4 个关节角度 → 虚拟腿长 L₀ + 倾角 φ₀ + 雅可比系数 A

    // 左腿
    double L_calf_angle  = state.joint_angles[0];
    double L_thigh_angle = params_.pi + state.joint_angles[1];
    auto vmc_L = vmc_forward_kinematics(L_calf_angle, L_thigh_angle, true);

    // 右腿
    double R_thigh_angle = state.joint_angles[2];
    double R_calf_angle  = params_.pi + state.joint_angles[3];
    auto vmc_R = vmc_forward_kinematics(R_calf_angle, R_thigh_angle, false);

    // ────── 2. LQR 增益调度 + 控制输出 ──────
    //  左右腿各自独立计算 u = K(L₀)·X

    auto lqr_L = lqr_gain_schedule(vmc_L.L0, state.state_vector);
    auto lqr_R = lqr_gain_schedule(vmc_R.L0, state.state_vector);

    // ────── 3. 综合推力 F + 扭矩 Tp (简化版,无离地检测/PID) ──────
    //  在实际 STM32 代码中,F 来自腿长 PID + 横滚补偿 + 重力补偿
    //  这里用简化的常量重力补偿 (100N/腿)
    const double gravity_comp = 100.0;
    double F_L  = gravity_comp;
    double F_R  = gravity_comp;
    double Tp_L = lqr_L.balance_Tp;
    double Tp_R = lqr_R.balance_Tp;

    // ────── 4. 雅可比逆矩阵: F/Tp → 4 个关节力矩 ──────
    auto [T_calf_L, T_thigh_L] = jacobian_torque_mapping(F_L, Tp_L, vmc_L.A_coeff, params_.link_ratio_K, true);
    auto [T_calf_R, T_thigh_R] = jacobian_torque_mapping(F_R, Tp_R, vmc_R.A_coeff, params_.link_ratio_K, false);

    // 限幅
    cmd.joint_torque[0] = clamp(T_calf_L,   -params_.joint_torque_max, params_.joint_torque_max);
    cmd.joint_torque[1] = clamp(T_thigh_L,  -params_.joint_torque_max, params_.joint_torque_max);
    cmd.joint_torque[2] = clamp(T_thigh_R,  -params_.joint_torque_max, params_.joint_torque_max);
    cmd.joint_torque[3] = clamp(T_calf_R,   -params_.joint_torque_max, params_.joint_torque_max);

    // ────── 5. 轮子力矩 → 电流 ──────
    double wheel_T_L = lqr_L.balance_T;
    double wheel_T_R = lqr_R.balance_T;

    cmd.wheel_current[0] = clamp( wheel_T_L * params_.torque_to_current, -params_.wheel_current_max, params_.wheel_current_max);
    cmd.wheel_current[1] = clamp(-wheel_T_R * params_.torque_to_current, -params_.wheel_current_max, params_.wheel_current_max);

    return cmd;
}

} // namespace sim_core
