
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Image_Transmission.h"

 void CAN_Task(void const * argument)
{
  TickType_t CAN_Task_SysTick = 0;
//	 DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[0],Motor_Save_Zero_Position);
//   osDelay(30);
//	 DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[1],Motor_Save_Zero_Position);
//   osDelay(30);
//   DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[2],Motor_Save_Zero_Position);
//   osDelay(30);
//	 DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[3],Motor_Save_Zero_Position);
//	


	for(;;)
  {
	
  CAN_Task_SysTick = osKernelSysTick();
		if(DM_8009_Motor[0].Data.State != 1 && DM_8009_Motor[1].Data.State != 1 && DM_8009_Motor[2].Data.State != 1 &&  DM_8009_Motor[3].Data.State != 1)
	{
	//所有关节电机使能		
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[0],Motor_Enable);
    osDelay(30);
	 DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[1],Motor_Enable);
  	osDelay(30);
    DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[2],Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[3],Motor_Enable);
	osDelay(30);
	}else {		
	 //当遥控器没拨到关机档时	
	if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 1){
// 		//测试转换器
// 		//当关节电机没转到安全位置时
 	 	if(Control_Info.Init.Joint_Init.IF_Joint_Init == 0){
// 			//让关节电机位置归零
	   		       DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,15.f,1.f,0);
	   		       DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,15.f,1.f,0);
        		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,15.f,1.f,0);	
        		   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,15.f,1.f,0);
 			
					
			
			
 		 	//让轮子失能
 			 FDCAN3_TxFrame.Header.Identifier = 0x200;					
 		 	 FDCAN3_TxFrame.Data[0] = 0;
 		     FDCAN3_TxFrame.Data[1] = 0;
 		 	 FDCAN3_TxFrame.Data[2] = 0;
 		 	 FDCAN3_TxFrame.Data[3] = 0;
 		 	 FDCAN3_TxFrame.Data[4] = 0;
 		 	 FDCAN3_TxFrame.Data[5] = 0;
 		 	 USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
// 	 //当整车初始化完成时
 	 }else{
// 		//开始给电机发送数据
// 					//让关节电机位置归零------测试用


//测试平衡模式
 		  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,Control_Info.L_Leg_Info.SendValue.T_Calf);
 	    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,Control_Info.L_Leg_Info.SendValue.T_Thigh);
      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,Control_Info.R_Leg_Info.SendValue.T_Thigh);	
      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0,0,Control_Info.R_Leg_Info.SendValue.T_Calf);	
			FDCAN3_TxFrame.Header.Identifier = 0x200;					
 		  FDCAN3_TxFrame.Data[0] = (uint8_t)(Control_Info.L_Leg_Info.SendValue.Current>>8);
 		  FDCAN3_TxFrame.Data[1] = (uint8_t)(Control_Info.L_Leg_Info.SendValue.Current);
 		  FDCAN3_TxFrame.Data[2] = (uint8_t)(Control_Info.R_Leg_Info.SendValue.Current>>8);
 		  FDCAN3_TxFrame.Data[3] = (uint8_t)(Control_Info.R_Leg_Info.SendValue.Current);
 		  FDCAN3_TxFrame.Data[4] = (uint8_t)(0)>>8;//预留
 		  FDCAN3_TxFrame.Data[5] = (uint8_t)(0);//预留
 		  USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
//		 

		}
	//当要关机时	 
	}else{	
		//全部断电
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,0);
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,0);
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,0);	
		 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0,0,0);	
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
	 
	 //500Hz发送 请保证所有任务osDelay(1)
	 
	 }	
		osDelay(1);
  }
 
}


