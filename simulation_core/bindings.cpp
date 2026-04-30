/**
 * @file    bindings.cpp
 * @brief   pybind11 绑定: 将 WheelLeggedDynamics 暴露为 Python 模块
 *
 * 编译后生成 wheel_legged_sim.<so|pyd>，可直接在 Python 中:
 *   from wheel_legged_sim import WheelLeggedDynamics, RobotState, MotorCommand
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "wheel_legged_dynamics.hpp"

namespace py = pybind11;
using namespace sim_core;

PYBIND11_MODULE(wheel_legged_sim, m) {
    m.doc() = "偏置并联轮腿机器人 VMC+LQR 仿真核心 (Sim-Core)";

    // RobotParams (可配置机械参数)
    py::class_<RobotParams>(m, "RobotParams")
        .def(py::init<>())
        .def_readwrite("thigh_link_len", &RobotParams::thigh_link_len)
        .def_readwrite("calf_link_len",  &RobotParams::calf_link_len)
        .def_readwrite("link_ratio_K",   &RobotParams::link_ratio_K)
        .def_readwrite("wheel_radius",   &RobotParams::wheel_radius)
        .def_readwrite("joint_torque_max", &RobotParams::joint_torque_max)
        .def_readwrite("wheel_current_max", &RobotParams::wheel_current_max);

    // RobotState (完整机器人状态)
    py::class_<RobotState>(m, "RobotState")
        .def(py::init<>())
        .def_readwrite("joint_angles",     &RobotState::joint_angles)
        .def_readwrite("joint_velocities", &RobotState::joint_velocities)
        .def_readwrite("state_vector",     &RobotState::state_vector)
        .def_readwrite("chassis_velocity", &RobotState::chassis_velocity)
        .def_readwrite("target_velocity",  &RobotState::target_velocity);

    // MotorCommand (输出)
    py::class_<MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readonly("joint_torque",  &MotorCommand::joint_torque)
        .def_readonly("wheel_current", &MotorCommand::wheel_current);

    // VmcResult
    py::class_<VmcResult>(m, "VmcResult")
        .def(py::init<>())
        .def_readonly("L0", &VmcResult::L0)
        .def_readonly("phi0", &VmcResult::phi0)
        .def_readonly("A_coeff", &VmcResult::A_coeff);

    // WheelLeggedDynamics (核心类)
    py::class_<WheelLeggedDynamics>(m, "WheelLeggedDynamics")
        .def(py::init<>())
        .def(py::init<const RobotParams&>())
        .def("compute_torques", &WheelLeggedDynamics::compute_torques,
             py::arg("state"),
             "核心入口: 给定状态 → 6个电机目标指令")
        .def("vmc_forward_kinematics", &WheelLeggedDynamics::vmc_forward_kinematics,
             py::arg("calf_angle"), py::arg("thigh_angle"), py::arg("is_left_leg"),
             "正运动学: 2个关节角度 → 虚拟腿长 L₀ + 倾角 φ₀")
        .def("lqr_gain_schedule", &WheelLeggedDynamics::lqr_gain_schedule,
             py::arg("L0"), py::arg("X"),
             "LQR 增益调度: K = f(L₀), u = K·X")
        .def("jacobian_torque_mapping", &WheelLeggedDynamics::jacobian_torque_mapping,
             py::arg("F"), py::arg("Tp"), py::arg("A_coeff"), py::arg("link_ratio"), py::arg("is_left_leg"),
             "雅可比逆矩阵: 虚拟力/扭矩 → 关节力矩")
        .def_property("params",
            [](const WheelLeggedDynamics& self) { return self.params(); },
            [](WheelLeggedDynamics& self, const RobotParams& p) { self.set_params(p); });
}
