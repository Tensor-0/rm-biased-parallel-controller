/**
  ******************************************************************************
  * @file           : sensor_fusion.h
  * @brief          : IMU + wheel odometry sensor fusion
  * @description    : Computes 6-DOF state vector from raw sensor data.
  ******************************************************************************
  */
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "Control_Task.h"
#include "control_io.h"

void SensorFusion_Measure_Update(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in);

#endif /* SENSOR_FUSION_H */
