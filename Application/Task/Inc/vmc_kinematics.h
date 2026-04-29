/**
  ******************************************************************************
  * @file           : vmc_kinematics.h
  * @brief          : VMC (Virtual Model Control) kinematics module
  * @description    : Forward kinematics: biased-parallel leg → simplified model.
  *                   Joint torque mapping: simplified forces → real joint torques.
  ******************************************************************************
  */
#ifndef VMC_KINEMATICS_H
#define VMC_KINEMATICS_H

#include "Control_Task.h"
#include "control_io.h"

void VMC_Joint_Angle_Offset(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);
void VMC_Calculate(Control_Info_Typedef *Control_Info);
void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);
void VMC_Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info);

#endif /* VMC_KINEMATICS_H */
