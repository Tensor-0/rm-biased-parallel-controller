/**
  ******************************************************************************
  * @file           : control_io.c
  * @brief          : 控制任务 I/O 边界 — 输入快照采集 + 输出命令包打包
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   这是全系统唯一能直接访问硬件全局变量的地方！
  *   - 采集快照: 1ms一次,把所有传感器数据复制到 control_input_snapshot_t
  *   - 打包命令: Control_Task 算完后,把 SendValue 拷贝到 g_motor_cmd
  *
  * 【为什么它特殊？】
  *   这是"海关": 算法层(Control_Task)不能直接碰硬件全局变量,
  *   防止并发访问带来的数据竞争。你只需要知道:
  *   - 所有读硬件: 从这里走 (Control_InputSnapshot_Update)
  *   - 所有写硬件: 从这里走 (Control_OutputPacket_Generate)
  *
  ******************************************************************************
  */

#include "control_io.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "bsp_adc.h"
#include "cmsis_os.h"

/* ====== 全局输出命令包 ======
   🐣 g_motor_cmd 是 Control_Task 和 CAN_Task 之间唯一的沟通桥梁。
   Control_Task 写入, CAN_Task 只读。两者在不同线程中运行,
   但由于写入和读取都是按值赋值(非指针),在 Cortex-M7 上对浮点值还算安全。
   未来可考虑改用 FreeRTOS 队列(Queue)保证严格数据一致性。 */
motor_command_packet_t g_motor_cmd = {0};

/**
 * Control_InputSnapshot_Update — 对所有传感器"拍照"
 *
 * 🐣 这个函数在 Control_Task 每个周期开始时被调用一次。
 *   把 DM_8009_Motor, Chassis_Motor, remote_ctrl, INS_Info 等的当前值
 *   一次性复制到快照结构体中。之后整个控制循环只看这个快照。
 *
 *   为什么不能直接访问？想象你一边看菜单一边厨师在改菜单——乱套了。
 *   快照 = "菜单打印出来的一份复印件",后续所有人都看复印件。
 */
void Control_InputSnapshot_Update(control_input_snapshot_t *in)
{
    /* [时间戳] 记录快照采集时刻的 FreeRTOS tick */
    in->tick = osKernelSysTick();

    /* [IMU数据] 直接整体拷贝 INS_Info (约44字节)
       🐣 INS_Task 也以 1kHz 在更新 INS_Info,两者频率一致。
       但这里有数据竞争风险(一条腿在写,一条腿在读),
       建议未来改用消息队列。当前工程中已验证无明显竞态。 */
    in->ins = INS_Info;

    /* [遥控器] 逐个通道拷贝
       🐣 remote_ctrl 在 USART 中断回调中被更新(非RTOS线程),
       所以这里逐字节拷贝,不用结构体赋值 */
    in->rc.ch[0] = remote_ctrl.rc.ch[0];
    in->rc.ch[1] = remote_ctrl.rc.ch[1];
    in->rc.ch[2] = remote_ctrl.rc.ch[2];
    in->rc.ch[3] = remote_ctrl.rc.ch[3];
    in->rc.ch[4] = remote_ctrl.rc.ch[4];
    in->rc.s[0]  = remote_ctrl.rc.s[0];
    in->rc.s[1]  = remote_ctrl.rc.s[1];

    /* [4个关节电机] 循环拷贝位置/速度/力矩/状态
       🐣 DM_8009_Motor 在 FDCAN2 中断回调中被更新,
       同样存在中断与RTOS线程的竞争风险 */
    for (int i = 0; i < 4; i++) {
        in->joint[i].position = DM_8009_Motor[i].Data.Position;
        in->joint[i].velocity = DM_8009_Motor[i].Data.Velocity;
        in->joint[i].torque   = DM_8009_Motor[i].Data.Torque;
        in->joint[i].state    = DM_8009_Motor[i].Data.State;
    }

    /* [2个驱动轮电机] */
    in->wheel[0].velocity = Chassis_Motor[0].Data.Velocity;
    in->wheel[1].velocity = Chassis_Motor[1].Data.Velocity;

    /* [电池电压] 通过 ADC 读取
       🐣 24V 电池经过分压电阻进入 ADC 通道,USER_ADC_Voltage_Update 完成采样+转换 */
    in->vbat = USER_ADC_Voltage_Update();
}

/**
 * Control_OutputPacket_Generate — 打包电机命令包
 *
 * 🐣 把 Control_Info 中的 SendValue 拷贝到 g_motor_cmd。
 *   只做赋值,不做任何计算。保证"该发的发了,不该改的没改"。
 *
 *   📦 关节顺序 (一定要对):
 *     [0]=左小腿 T_Calf, [1]=左大腿 T_Thigh, [2]=右大腿 T_Thigh, [3]=右小腿 T_Calf
 *   ⚠ 注意右腿的 T_Thigh(大腿)和 T_Calf(小腿)顺序跟左腿不同!
 */
void Control_OutputPacket_Generate(const Control_Info_Typedef *ctrl,
                                   motor_command_packet_t *out)
{
    /* 📦 关节力矩: 按"左小腿→左大腿→右大腿→右小腿"顺序打包 */
    out->joint_torque[0] = ctrl->L_Leg_Info.SendValue.T_Calf;
    out->joint_torque[1] = ctrl->L_Leg_Info.SendValue.T_Thigh;
    out->joint_torque[2] = ctrl->R_Leg_Info.SendValue.T_Thigh;
    out->joint_torque[3] = ctrl->R_Leg_Info.SendValue.T_Calf;

    /* 📦 驱动轮电流 */
    out->wheel_current[0] = ctrl->L_Leg_Info.SendValue.Current;
    out->wheel_current[1] = ctrl->R_Leg_Info.SendValue.Current;

    /* 📦 状态标志: 告诉 CAN_Task 当前是平衡/虚弱/初始化 */
    out->chassis_situation = (uint8_t)ctrl->Chassis_Situation;
    out->joint_init_done   = ctrl->Init.Joint_Init.IF_Joint_Init;

    /* 📦 电机激活标志: 用于 CAN_Task 判断是否要发动力矩
       🐣 来源: remote_ctrl.rc.s[1] 的映射 (在 control_io.c 中转换, CAN_Task 不需要知道遥控器) */
    out->motor_active = (uint8_t)(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 1);

    /* 📦 序号+时间戳: seq++用于检测丢包, tick 记录打包时刻 */
    out->seq++;
    out->tick = osKernelSysTick();
}
