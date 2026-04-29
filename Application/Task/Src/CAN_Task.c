
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Image_Transmission.h"
#include "control_io.h"  /* I/O边界：读取 g_motor_cmd，不再直接访问 Control_Info/INS_Info/remote_ctrl */

/**
  * @brief  CAN_Task — 电机CAN通信线程 (1kHz)
  * @note   本任务只通过 control_io.h 的 g_motor_cmd 获取控制指令，
  *         不再直接依赖 Control_Task.h / INS_Task.h / Remote_Control.h。
  */
 void CAN_Task(void const * argument)
{
  TickType_t CAN_Task_SysTick = 0;

	for(;;)
  {

  CAN_Task_SysTick = osKernelSysTick();
		if(DM_8009_Motor[0].Data.State != 1 && DM_8009_Motor[1].Data.State != 1 && DM_8009_Motor[2].Data.State != 1 &&  DM_8009_Motor[3].Data.State != 1)
	{
		/* 关节电机未使能 → 发送使能命令 */
	DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[0], Motor_Enable);
    osDelay(30);
	 DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[1], Motor_Enable);
  	osDelay(30);
    DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[2], Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[3], Motor_Enable);
	osDelay(30);
	}else {
	 /* 电机已使能，根据 g_motor_cmd.motor_active 决定发送模式 */
	if(g_motor_cmd.motor_active != 0){
		/* 关节初始化未完成 → 归零模式 */
 	 	if(g_motor_cmd.joint_init_done == 0){
	   		       DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 10.f, 1.f, 0);
	   		       DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 10.f, 1.f, 0);
        		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 10.f, 1.f, 0);
        		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 10.f, 1.f, 0);

 		 /* 初始化期间轮子失能 */
 			 FDCAN3_TxFrame.Header.Identifier = 0x200;
 		 	 FDCAN3_TxFrame.Data[0] = 0;
 		     FDCAN3_TxFrame.Data[1] = 0;
 		 	 FDCAN3_TxFrame.Data[2] = 0;
 		 	 FDCAN3_TxFrame.Data[3] = 0;
 		 	 FDCAN3_TxFrame.Data[4] = 0;
 		 	 FDCAN3_TxFrame.Data[5] = 0;
 		 	 USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

 	 }else{
		/* 正常运行模式：从 g_motor_cmd 获取电机指令 */
 		  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 0, 0, g_motor_cmd.joint_torque[0]);
 		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 0, 0, g_motor_cmd.joint_torque[1]);
 		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 0, 0, g_motor_cmd.joint_torque[2]);
 		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 0, 0, g_motor_cmd.joint_torque[3]);
 		FDCAN3_TxFrame.Header.Identifier = 0x200;
 		  FDCAN3_TxFrame.Data[0] = (uint8_t)(g_motor_cmd.wheel_current[0] >> 8);
 		  FDCAN3_TxFrame.Data[1] = (uint8_t)(g_motor_cmd.wheel_current[0]);
 		  FDCAN3_TxFrame.Data[2] = (uint8_t)(g_motor_cmd.wheel_current[1] >> 8);
 		  FDCAN3_TxFrame.Data[3] = (uint8_t)(g_motor_cmd.wheel_current[1]);
 		  FDCAN3_TxFrame.Data[4] = 0;
 		  FDCAN3_TxFrame.Data[5] = 0;
 		  USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

		}
	/* 关机状态：全部断电 */
	}else{
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 0, 0, 0);
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 0, 0, 0);
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 0, 0, 0);
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 0, 0, 0);
		  FDCAN3_TxFrame.Header.Identifier = 0x200;
		  FDCAN3_TxFrame.Data[0] = 0;
		  FDCAN3_TxFrame.Data[1] = 0;
		  FDCAN3_TxFrame.Data[2] = 0;
		  FDCAN3_TxFrame.Data[3] = 0;
		  FDCAN3_TxFrame.Data[4] = 0;
		  FDCAN3_TxFrame.Data[5] = 0;
		  USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

	}
	}

	 if(CAN_Task_SysTick % 2 == 0){
	 /* 预留500Hz子任务入口 */
	 }
		osDelay(1);
  }

}


