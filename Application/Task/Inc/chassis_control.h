/**
  ******************************************************************************
  * @file           : chassis_control.h
  * @brief          : High-level chassis motion control
  * @description    : Movement, height, roll, and leg-length control logic.
  ******************************************************************************
  */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "Control_Task.h"
#include "control_io.h"

void Chassis_Move_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);
void Chassis_Height_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);
void Chassis_Roll_Control(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);
void Leg_Length_Control(Control_Info_Typedef *Control_Info);

#endif /* CHASSIS_CONTROL_H */
