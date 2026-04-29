/**
  ******************************************************************************
  * @file           : lqr_controller.h
  * @brief          : Gain-scheduled LQR balance controller
  * @description    : K = f(L0) polynomial, X = Target - Measure, u = -K·X
  ******************************************************************************
  */
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "Control_Task.h"

void LQR_K_Update(Control_Info_Typedef *Control_Info);
void LQR_X_Update(Control_Info_Typedef *Control_Info);
void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info);
/* Note: K11-K26 gain arrays are defined in the module .c file */

#endif /* LQR_CONTROLLER_H */
