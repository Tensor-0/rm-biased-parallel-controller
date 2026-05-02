/**
  ******************************************************************************
  * @file           : Robot_Config.h
  * @brief          : 机器人全局参数集中配置文件
  * @description    : 所有机械参数、控制参数、PID参数集中在此。
  *                   换车/调参只需改这一个文件，无需深入算法代码。
  *
  * ======  新手必读 ======
  * 如果你要调参数，找这个文件就够了。所有"魔法数字"都在这里。
  * 格式：[参数名] (单位) | 默认值 | 说明
  ******************************************************************************
  */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ======================== 机械参数 ======================== */

/* [驱动轮半径] (米 m) | 0.055 | M3508 轮毂电机 + 轮胎 */
#define CONF_WHEEL_RADIUS           0.055f

/* [两轮机械间距] (米 m) | 0.4157 | 左右轮中心之间的距离 */
#define CONF_WHEEL_DISTANCE         0.4157f

/* [大腿连杆长度 AH] (米 m) | 0.118114 | 动力长后杆 */
#define CONF_THIGH_LINK_LEN         0.118114f

/* [小腿连杆长度 AD] (米 m) | 0.100 | 动力短前杆 */
#define CONF_CALF_LINK_LEN          0.100f

/* [连杆长度比 K=AD/AH] | 0.465116 | 比值越小同角度腿长变化越大 */
#define CONF_LEG_LINK_RATIO         0.465116f

/* [机体质心到转轴距离] (米 m) | 0.0011 | 越近越不容易"点头" */
#define CONF_BODY_COM_OFFSET        0.0011f

/* ======================== 质量参数 ======================== */

/* [机体质量] (千克 kg) | 18.12 | 不含腿和轮子 */
#define CONF_BODY_MASS              18.12f

/* [虚拟腿(摆杆)质量] (千克 kg) | 1.12 | 单条虚拟简化腿 */
#define CONF_LEG_MASS               1.12f

/* [驱动轮质量] (千克 kg) | 0.88 | 单个 M3508+轮胎 */
#define CONF_WHEEL_MASS             0.88f

/* [半车重] (牛顿 N) | 100 | 用于重力补偿 (≈(M+2mp+2mw)×g/2) */
#define CONF_GRAVITY_COMPENSATION   100.f

/* ======================== 控制频率 ======================== */

/* [控制周期] (秒 s) | 0.001 | 1kHz = 1ms 一次 */
#define CONF_CTRL_DT                0.001f

/* ======================== 腿长范围 ======================== */

/* [最小虚拟腿长] (米 m) | 0.14 | 低底盘/收缩状态 */
#define CONF_LEG_LENGTH_MIN         0.14f

/* [最大虚拟腿长] (米 m) | 0.32 | 机械限位 */
#define CONF_LEG_LENGTH_MAX         0.32f

/* [低腿长目标] (米 m) | 0.14 | 公路模式 */
#define CONF_LEG_LENGTH_LOW         0.14f

/* [高腿长目标] (米 m) | 0.20 | 越野模式 */
#define CONF_LEG_LENGTH_HIGH        0.20f

/* ======================== 速度与力矩限制 ======================== */

/* [最大底盘速度] (米/秒 m/s) | 1.6 | ≈5.8km/h */
#define CONF_MAX_CHASSIS_SPEED      1.6f

/* [关节力矩上限] (牛·米 N·m) | 15 | 防止烧电机 */
#define CONF_JOINT_TORQUE_MAX       15.f

/* [驱动轮电流上限] (毫安 mA) | 10000 | 防止烧电机 */
#define CONF_WHEEL_CURRENT_MAX      10000

/* [力矩→电流系数] | 1200 | 经验换算系数 */
#define CONF_TORQUE_TO_CURRENT      1200.f

/* [轮速→角速度减速比] | 15 | 电机15圈轮子1圈 */
#define CONF_WHEEL_REDUCTION_RATIO  15.f

/* ======================== 传感器融合参数 ======================== */

/* [融合滤波器系数 α] | 0.8 | Fusion = α×Predict + (1-α)×Body */
#define CONF_FUSION_ALPHA           0.8f

/* [预测加速度衰减] | 0.155 | IMU到旋转中心的距离(米),用于向心加速度计算 */ 
#define CONF_ACCEL_CENTRIPETAL_R    0.155f

/* ======================== 着地检测参数 ======================== */

/* [着地阈值] (牛顿 N) | 22 | FN < 22 判定为离地 */
#define CONF_SUPPORT_FN_THRESHOLD   22.f

/* [默认支持力] (牛顿 N) | 100 | 非平衡状态下的假值 */
#define CONF_SUPPORT_FN_DEFAULT     100.f

/* ======================== 横滚控制参数 ======================== */

/* [IMU Roll角零偏] (弧度 rad) | -0.0291 | 安装偏差补偿 */
#define CONF_ROLL_IMU_OFFSET        -0.0291f

/* [横滚角滤波速率] | 0.003 | 一阶滤波的平滑系数 */
#define CONF_ROLL_FILTER_RATE       0.003f

/* ======================== 底盘控制斜坡速率 ======================== */

/* [前进响应速率] | 0.001 | 越小起步越平滑 */
#define CONF_RAMP_VELOCITY          0.001f

/* [刹车响应速率] | 0.002 | 比起步稍快 */
#define CONF_RAMP_BRAKE             0.002f

/* [转向响应速率] | 0.2 | 适中 */
#define CONF_RAMP_YAW               0.2f

/* [高度切换速率] | 0.0003 | 非常慢,防止切换时摔倒 */
#define CONF_RAMP_HEIGHT            0.00030f

/* [腿长控制速率] | 0.35 | 适中 */
#define CONF_RAMP_LEG_LENGTH        0.35f

/* ======================== 遥控器映射 ======================== */

/* [摇杆→速度映射系数] | 0.00242 | ch值[-660,660]→速度[-1.6,1.6] */
#define CONF_RC_VELOCITY_SCALE      0.00242f

/* ======================== 初始化超时 ======================== */

/* [关节初始化超时] (毫秒 ms) | 5000 | 5秒内未完成则判定卡死 */
#define CONF_INIT_TIMEOUT_MS        5000

/* ======================== 电池电压 ======================== */

/* [低电压警告阈值] (伏特 V) | 22 | 6S锂电池最低安全电压 */
#define CONF_BAT_LOW_THRESHOLD      22.f

#endif /* ROBOT_CONFIG_H */
