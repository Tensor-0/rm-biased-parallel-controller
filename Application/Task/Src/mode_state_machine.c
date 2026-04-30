/**
  ******************************************************************************
  * @file           : mode_state_machine.c
  * @brief          : 模式状态机 + PID 初始化 — 机器人的"开机自检"和"档位切换"
  *
  * ====== 🐣 新手必读 ======
  *
  * 【这个文件在干什么？】
  *   机器人有三种状态: 关机(WEAK)、初始化归位、平衡(BALANCE)。
  *   这个文件负责在这三种状态之间切换,并在启动时给所有 PID 控制器
  *   赋予正确的参数。你可以理解为"开电源→挂挡→踩油门"的过程。
  *
  * 【核心职责】
  *   1. PID_Init: 给 6 个 PID 控制器装填 Kp/Ki/Kd 参数
  *   2. Check_Low_Voltage: 监测电池电压
  *   3. Mode_Update: 遥控器开关→控制状态机切换
  *
  * 【前置知识】
  *   - PID 参数含义: Kp=比例(用力大小), Ki=积分(消除稳态误差), Kd=微分(防抖)
  *   - PID_PARAM 宏: 把 7 个魔法数字变成有名字的格式
  *   - 状态机: 一种"根据条件切换行为"的设计模式
  *
  ******************************************************************************
  */

#include "mode_state_machine.h"
#include "PID.h"
#include "Robot_Config.h"
#include "cmsis_os.h"

/* 🐣 (v3.1) 初始化超时计数器 — 防止关节卡死导致永久阻塞 */
static uint32_t init_timeout_tick = 0;

/* ====== PID 参数定义 ======
   🐣 PID_PARAM(Kp, Ki, Kd, Alpha, Deadband, LimitI, LimitO) 的 7 个参数:
   Kp    = 比例增益:    误差放大倍数,决定"反应有多剧烈"
   Ki    = 积分增益:    累计误差放大倍数,消除"你总觉得差一点点"的问题
   Kd    = 微分增益:    误差变化率放大倍数,"D项就像在蜂蜜里移动,柔和不震荡"
   Alpha = 微分滤波系数: 对D项做低通滤波,防止噪声放大 (0=无滤波)
   Deadband = 死区:     误差绝对值小于此值时 PID 不计算 (防止微动振动)
   LimitI   = 积分限幅: 防止积分饱和(windup)导致超调
   LimitO   = 输出限幅: 最终输出不能超过这个范围
*/

#define PID_PARAM(Kp,Ki,Kd,A,Db,Li,Lo) { (Kp), (Ki), (Kd), (A), (Db), (Li), (Lo) }

/* [腿长PID参数] 控制目标: 虚拟腿长跟踪指定值
   Kp=1300 很高! 因为腿长差1mm都要用力推回去 */
static float PID_Leg_Length_F_Param[7]  = PID_PARAM(1300.f, 1.f,  60000.f, 0.f, 0.f, 10.f, 100.f);

/* [横滚补偿力PID] 控制目标: 机身保持水平
   Kp=50, Kd=25 让补偿既快速又不太过 */
static float PID_Leg_Roll_F_Param[7]    = PID_PARAM(50.f,   0.f,  25.f,    0.f, 0.f, 0.1f, 50.f);

/* [防劈叉PID] 控制目标: 两条腿角度保持同步
   Kp=300 强力纠正,防止两条腿各走各的 */
static float PID_Leg_Coordinate_param[7]= PID_PARAM(300.f,  0.f,  20.0f,  0.f, 0.f, 0.f,  50);

/* [偏航位置PID] 控制目标: 机头指向指定角度
   Kp=4.4 比较温和,因为转太猛会破坏平衡 */
static float PID_Yaw_P_pama[7]          = PID_PARAM(4.4f,   0.f,  60.f,   0,   0,   200,  500);

/* [偏航速度PID] 控制目标: 机头转动速度跟踪上级PID输出
   Kp=0.25, Kd=0.4 偏弱,作为内环起到"平滑转速"作用 */
static float PID_Yaw_V_pama[7]          = PID_PARAM(0.25f,  0,    0.4f,   0,   0,   200,  70);

/* ====== 全局 PID 实例 ======
   🐣 这些都是"活"的控制器,每个都有内部状态(误差积分、上次误差等)。
   它们被定义在这里(而非 Control_Task.c),因为这里是"PID 的集中营"。
   其他模块通过 extern 声明来使用它们。 */

/* [腿部协调控制器] 负责防止"劈叉" */
PID_Info_TypeDef PID_Leg_Coordinate;

/* [腿长控制器×2] 左腿和右腿各一个,分别控制各自的虚拟腿长 */
PID_Info_TypeDef PID_Leg_length_F[2];

/* [横滚补偿力控制器] 负责抵抗左右倾斜 */
PID_Info_TypeDef PID_Leg_Roll_F;

/* [偏航控制器×2] 串级PID: [0]=外环(位置), [1]=内环(速度) */
PID_Info_TypeDef PID_Yaw[2];

/**
 * Mode_Init — 一次性初始化所有 PID 控制器
 *
 * 🐣 这个函数在 Control_Task 启动时只运行一次。
 *    类似开机自检——把 Kp/Ki/Kd 参数"写入"到 PID 控制器的内部记忆里。
 *    PID_POSITION = 位置式 PID (每次算输出,不是增量式)
 */
void Mode_Init(Control_Info_Typedef *Control_Info)
{
    PID_Init(&PID_Leg_length_F[0], PID_POSITION, PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_length_F[1], PID_POSITION, PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_Roll_F,      PID_POSITION, PID_Leg_Roll_F_Param);
    PID_Init(&PID_Leg_Coordinate,  PID_POSITION, PID_Leg_Coordinate_param);
    PID_Init(&PID_Yaw[0],          PID_POSITION, PID_Yaw_P_pama);
    PID_Init(&PID_Yaw[1],          PID_POSITION, PID_Yaw_V_pama);
}

/**
 * Mode_Check_Low_Voltage — 读取电池电压
 *
 * 🐣 当前只做电压存储,不做报警。电压值从快照中获取,
 *    以保持 I/O 隔离(不直接读 ADC)。
 */
void Mode_Check_Low_Voltage(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* [电池电压] (伏特 V) | 正常: [22, 25.2] | <22V需充电
       🐣 VDC 存在 Control_Info 里,供后续状态显示/报警使用 */
    Control_Info->VDC = in->vbat;
}

/**
 * Mode_Update — 核心状态机: 根据遥控器开关切换控制模式
 *
 * 🐣 这是机器人唯一的"档位切换"逻辑:
 *
 *  遥控器 s[1] 档位:
 *   ├── 3 (下拨最底) = 初始化 + 平衡模式
 *   ├── 1 (下拨中间) = 高腿长模式
 *   ├── 2 (上拨中间) = 关机 (所有电机断电)
 *   └── 0 (上拨最顶) = 关机
 *
 *   状态转移:
 *   关机 → (s[1]=3/1) → 初始化 → 4关节到位 → CHASSIS_BALANCE(平衡)
 *   平衡 → (s[1]=2/0) → CHASSIS_WEAK(虚弱,电机断电)
 *
 *   ⚠ 关键安全逻辑:
 *   - 只有 4 个关节同时到达安全位置,才算"初始化完成"
 *   - 初始化完成前,系统状态是 CHASSIS_WEAK(不输出控制量)
 *   - 关机时立即清零所有初始化标志,防止残留状态意外触发
 */
void Mode_Update(Control_Info_Typedef *Control_Info, const control_input_snapshot_t *in)
{
    /* 🐣 步骤1: 检测遥控器档位
       短路求值: if(s[1]==3) 或 if(s[1]==1/2), 两个条件满足任意一个就进入 */
    if (in->rc.s[1] == 3 || in->rc.s[1]) {
        /* s[1]==3(初始化) 或 s[1]==1(高腿长) → 允许初始化 */
        Control_Info->Init.IF_Begin_Init = 1;
        /* 🐣 如果在初始化流程中拨到位置2(关机),立即清零初始化标志 */
        if (in->rc.s[1] == 2) {
            Control_Info->Init.IF_Begin_Init = 0;
            Control_Info->Chassis_Situation = CHASSIS_WEAK;
        }
    } else {
        /* s[1]==0 或 s[1]==2 → 关机
           🐣 如果当前是平衡状态,要把它打回虚弱状态 */
        Control_Info->Init.IF_Begin_Init = 0;
        if (Control_Info->Chassis_Situation == CHASSIS_BALANCE)
            Control_Info->Chassis_Situation = CHASSIS_WEAK;
    }

    /* 🐣 步骤2: 如果允许初始化 且 当前是虚弱状态 → 开始初始化流程 */
    if (Control_Info->Init.IF_Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        /* 步骤2a: 检查 4 个关节是否都在安全位置 */
        if (Control_Info->Init.Joint_Init.IF_Joint_Init == 0) {
            /* [左小腿] 安全位置: position < 0 (机械限位的收缩方向) */

            /* 🐣 (v3.1) 初始化超时检测: 如果初始化超过5秒还没完成,强制退出 */
            if (init_timeout_tick == 0) {
                init_timeout_tick = osKernelSysTick();
            } else if ((osKernelSysTick() - init_timeout_tick) > CONF_INIT_TIMEOUT_MS) {
                Control_Info->Init.IF_Begin_Init = 0;
                Control_Info->Chassis_Situation = CHASSIS_WEAK;
                init_timeout_tick = 0;
                return;
            }
            if (in->joint[0].position < 0.0f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;

            /* [左大腿] 安全位置: position 在 [-0.21, -0.005] 范围内 */
            if (in->joint[1].position > -0.21f && in->joint[1].position < -0.005f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;

            /* [右大腿] 安全位置: position 在 [0.01, 0.20] 范围内 */
            if (in->joint[2].position < 0.20f && in->joint[2].position > 0.01f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;

            /* [右小腿] 安全位置: position > 0 (与左小腿对称,方向相反) */
            if (in->joint[3].position > -0.0f)
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 1;
            else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;

            /* 🐣 步骤2b: 当 4 个标志全部为 1 → 初始化完成！
               把 4 个 bool 值加起来,和为 4 就是全部到位 */
            if (Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] +
                Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] == 4) {
                Control_Info->Init.Joint_Init.IF_Joint_Init = 1;
            } else {
                Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
            }
            /* 初始化期间,底盘位置清零(防止上一周期的累积误差) */
            Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
            Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
        } else if (Control_Info->Init.Joint_Init.IF_Joint_Init == 1) {
            /* 🐣 步骤2c: 初始化完成 → 进入平衡模式!
               CHASSIS_BALANCE 标志一旦置 1, LQR 控制回路正式开始工作 */
            Control_Info->Chassis_Situation = CHASSIS_BALANCE;
            Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
            Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
        }
    }

    /* 🐣 步骤3: 如果不再允许初始化 → 清零所有初始化标志
       这是"安全退出"逻辑,保证状态机不会卡在中间状态 */
    if (Control_Info->Init.IF_Begin_Init == 0 && Control_Info->Chassis_Situation == CHASSIS_WEAK) {
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
        Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
    }
}
