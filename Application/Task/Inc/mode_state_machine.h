/**
  ******************************************************************************
  * @file           : mode_state_machine.h
  * @brief          : System initialization and mode state machine
  * @description    : Low-voltage check, mode switching, PID initialization.
  ******************************************************************************
  */
#ifndef MODE_STATE_MACHINE_H
#define MODE_STATE_MACHINE_H

#include "Control_Task.h"
#include "control_io.h"

void Mode_Init(Control_Info_Typedef *Control_Info);
void Mode_Check_Low_Voltage(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);
void Mode_Update(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);

#endif /* MODE_STATE_MACHINE_H */
