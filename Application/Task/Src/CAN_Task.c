/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN 通信线程 — 把算好的电机指令发到 CAN 总线上
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   这是机器人的"传令兵"。它不参与任何计算——只负责从 g_motor_cmd
  *   命令包里读出已经算好的电机指令,然后通过 CAN 总线发送给 6 个电机
  *   (4个关节 DM8009 + 2个轮子 M3508)。
  *
  * 【核心职责】
  *   1. 电机使能: 上电后等电机自检完成,发送使能命令
  *   2. 初始化归零: 使能后关节未初始化时,发送位置归零命令
  *   3. 正常运行: 从 g_motor_cmd 中读取力矩/电流,发送到 CAN 总线
  *   4. 关机: 遥控器拨到关机档,全部电机扭矩清零
  *
  * 【前置知识】
  *   - FDCAN: 灵活数据速率 CAN, STM32H7 的高速 CAN 外设
  *   - DM_Motor_CAN_TxMessage: 大喵电机(妙德)的 CAN 协议发送函数
  *   - g_motor_cmd: Control_Task 计算好的电机命令包(本线程只读)
  *
  *   📦 CAN 总线分布:
  *   - FDCAN2: 4个 DM8009 关节电机 (ID 0,1,2,3)
  *   - FDCAN3: 2个 M3508 驱动轮电机 (ID 0x201, 0x202)
  *
  ******************************************************************************
  */

#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Image_Transmission.h"
#include "control_io.h"  /* I/O边界：只读取 g_motor_cmd */

/**
 * CAN_Task — 电机CAN通信线程 (1kHz)
 *
 * 🐣 注意: 本任务通过 osDelay(1) 也是 1kHz 频率,与 Control_Task 同频。
 *   但它不依赖精确频率——只是"有命令就发,没命令就等着"。
 *   g_motor_cmd.motor_active 决定了当前是初始化/运行/关机状态。
 */
void CAN_Task(void const * argument)
{
    TickType_t CAN_Task_SysTick = 0;

    for (;;) {
        CAN_Task_SysTick = osKernelSysTick();

        /* 🐣 步骤1: 检查电机是否已使能
           DM8009 电机的 State 字段: 0=未使能, 1=已使能
           如果 4 个电机中有一个没使能 → 发送使能命令(一次性)
           使能后 osDelay(30ms) 等待电机响应,避免连续发送导致总线拥塞 */
        if (DM_8009_Motor[0].Data.State != 1 && DM_8009_Motor[1].Data.State != 1 &&
            DM_8009_Motor[2].Data.State != 1 && DM_8009_Motor[3].Data.State != 1) {

            DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[0], Motor_Enable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[1], Motor_Enable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[2], Motor_Enable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[3], Motor_Enable);
            osDelay(30);

        } else {
            /* 🐣 电机已使能,根据 g_motor_cmd.motor_active 决定发送模式:
               motor_active=1 (s[1]=3/1): 初始化或运行
               motor_active=0 (s[1]=2/0): 关机 */

            if (g_motor_cmd.motor_active != 0) {

                /* 🐣 关节初始化未完成 → 归零模式
                   joint_init_done=0 时, Control_Task 的状态机还在等 4 个关节归位。
                   此期间发送"位置归零"命令(Position=0, KP=10, KD=1, Torque=0)
                   驱动电机缓慢回到安装零位,同时轮子断电(防止乱转)。 */
                if (g_motor_cmd.joint_init_done == 0) {
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 10.f, 1.f, 0);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 10.f, 1.f, 0);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 10.f, 1.f, 0);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 10.f, 1.f, 0);

                    /* 初始化期间轮子失能 (电流=0) */
                    FDCAN3_TxFrame.Header.Identifier = 0x200;
                    FDCAN3_TxFrame.Data[0] = 0;  FDCAN3_TxFrame.Data[1] = 0;
                    FDCAN3_TxFrame.Data[2] = 0;  FDCAN3_TxFrame.Data[3] = 0;
                    FDCAN3_TxFrame.Data[4] = 0;  FDCAN3_TxFrame.Data[5] = 0;
                    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

                } else {
                    /* 🐣 正常运行模式: 从 g_motor_cmd 获取电机指令 */

                    /* 📦 关节电机: 以 MIT 模式发送(位置=0,速度=0,KP=0,KD=0)
                       这意味着电机工作在"纯力矩模式"——只接受力矩指令,
                       位置和速度闭环由外部(Control_Task)完成。
                       力矩值来自 g_motor_cmd.joint_torque[4] */
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 0, 0, g_motor_cmd.joint_torque[0]);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 0, 0, g_motor_cmd.joint_torque[1]);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 0, 0, g_motor_cmd.joint_torque[2]);
                    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 0, 0, g_motor_cmd.joint_torque[3]);

                    /* 📦 驱动轮电机: CAN ID=0x200, 数据格式:
                       2个int16=4个字节,高字节在前
                       Data[0:1] = 左轮电流高/低字节
                       Data[2:3] = 右轮电流高/低字节
                       Data[4:5] = 预留(0) */
                    FDCAN3_TxFrame.Header.Identifier = 0x200;
                    FDCAN3_TxFrame.Data[0] = (uint8_t)(g_motor_cmd.wheel_current[0] >> 8);
                    FDCAN3_TxFrame.Data[1] = (uint8_t)(g_motor_cmd.wheel_current[0]);
                    FDCAN3_TxFrame.Data[2] = (uint8_t)(g_motor_cmd.wheel_current[1] >> 8);
                    FDCAN3_TxFrame.Data[3] = (uint8_t)(g_motor_cmd.wheel_current[1]);
                    FDCAN3_TxFrame.Data[4] = 0;
                    FDCAN3_TxFrame.Data[5] = 0;
                    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
                }

            } else {
                /* 🐣 关机状态: 全部电机断电 (力矩=0)
                   为什么要发力矩=0而不是不发？因为不发的话电机可能保持
                   上一帧的力矩——如果不发0,机器人会"定住不动"而不是"松垮垮地倒下"。 */
                DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 0, 0, 0);
                DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 0, 0, 0);
                DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 0, 0, 0);
                DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 0, 0, 0);

                FDCAN3_TxFrame.Header.Identifier = 0x200;
                FDCAN3_TxFrame.Data[0] = 0;  FDCAN3_TxFrame.Data[1] = 0;
                FDCAN3_TxFrame.Data[2] = 0;  FDCAN3_TxFrame.Data[3] = 0;
                FDCAN3_TxFrame.Data[4] = 0;  FDCAN3_TxFrame.Data[5] = 0;
                USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
            }
        }

        /* 预留: 每 2 个 tick 一次的子任务 (500Hz) */
        if (CAN_Task_SysTick % 2 == 0) {
        }
        osDelay(1);
    }
}
