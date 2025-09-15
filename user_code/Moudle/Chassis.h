#ifndef CHASSIS_H
#define CHASSIS_H

#include "system_config.h"
#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Motor.h"
#include "Pid.h"

// 云台死区限制宏
#define gimbal_deadband_limit(input, output, dealine)        \
    {                                                        \
        if ((input) > (dealine) || (input) < -(dealine))     \
        {                                                    \
            (output) = (input);                              \
        }                                                    \
        else                                                 \
        {                                                    \
            (output) = 0;                                    \
        }                                                    \
    }

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// 初始yaw轴角度
#define INIT_YAW_SET 0.0f

// 云台灵敏度设置
#define CHASSIS_VX_GIMBAL_SEN 0.006f     // 前进灵敏度
#define CHASSIS_VY_GIMBAL_SEN 0.005f     // 左右灵敏度
#define CHASSIS_WZ_GIMBAL_SEN 0.01f      // 旋转灵敏度

// 一阶滤波参数
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

// 云台死区
#define CHASSIS_GIMBAL_DEADLINE 10

// 电机速度到底盘速度的转换系数
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

// 电机到中心距离
#define MOTOR_DISTANCE_TO_CENTER 0.46f

// 底盘任务控制间隔
#define CHASSIS_CONTROL_TIME_MS 2          // 2ms
#define CHASSIS_CONTROL_TIME 0.002f        // 0.002s
#define CHASSIS_CONTROL_FREQUENCE 500.0f   // 500Hz

// 底盘电机最大CAN发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

// 云台控制模式定义
#define GIMBAL_CONTROL_FOLLOW 1        // 跟随模式
#define GIMBAL_CONTROL_NO_FOLLOW 2     // 不跟随模式
#define GIMBAL_CONTROL_ZERO_FORCE 3    // 无力模式

// 电机RPM到速度向量的转换系数
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

// 速度限制
#define MAX_WHEEL_SPEED 2.0f                   // 单个底盘电机最大速度
#define NORMAL_MAX_CHASSIS_SPEED_X 7.0f        // 最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.5f        // 最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 8.0f        // 最大旋转速度

// 电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

// 电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f  // 2*PI/8192

// 底盘开环模式下的缩放系数
#define CHASSIS_OPEN_GIMBAL_SCALE 10

// 底盘电机速度环PID参数
#define MOTIVE_MOTOR_SPEED_PID_KP 8000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 20000.0f

// 底盘旋转跟随PID参数
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 300.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 20000.0f

// 底盘行为模式枚举
typedef enum
{
    CHASSIS_ZERO_FORCE,                  // 底盘无力模式，电机电流为0
    CHASSIS_NO_MOVE,                     // 底盘不动模式，但存在抵抗力
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台模式
    CHASSIS_NO_FOLLOW_YAW,               // 底盘不跟随云台模式
    CHASSIS_OPEN,                        // 底盘开环模式，云台直接控制电机电流
} chassis_behaviour_e;

// 底盘控制模式枚举
typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台控制模式
    CHASSIS_VECTOR_NO_FOLLOW_YAW,      // 底盘不跟随云台控制模式
    CHASSIS_VECTOR_RAW,                // 底盘原始控制模式
} chassis_mode_e;

// 速度结构体
struct speed_t
{
    fp32 speed;        // 当前速度
    fp32 speed_set;    // 设定速度
    fp32 max_speed;    // 最大速度
    fp32 min_speed;    // 最小速度
};

// 底盘类定义
class Chassis {
public:
    // 底盘行为和控制状态机
    chassis_behaviour_e chassis_behaviour_mode;
    chassis_behaviour_e last_chassis_behaviour_mode;
    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    // 底盘电机对象
    M3508_motor chassis_motive_motor[4];

    // 一阶滤波器
    First_order_filter chassis_cmd_slow_set_vx;
    First_order_filter chassis_cmd_slow_set_vy;

    // PID控制器
    Pid chassis_wz_angle_pid;

    // 速度控制
    speed_t x;  // 前后方向
    speed_t y;  // 左右方向
    speed_t z;  // 旋转方向

    // 角度相关
    fp32 chassis_relative_angle;      // 底盘与云台的相对角度(rad)
    fp32 chassis_relative_angle_set;  // 相对角度设定值
    fp32 chassis_yaw_set;             // yaw角度设定值

    // 底盘姿态角度
    fp32 chassis_yaw;    // yaw角度
    fp32 chassis_pitch;  // pitch角度
    fp32 chassis_roll;   // roll角度


    // 公共方法
    void init();                // 初始化底盘
    void set_mode();            // 设置底盘模式
    void feedback_update();     // 更新反馈数据
    void set_contorl();         // 设置控制值
    void solve();               // 解算控制数据
    void output();              // 输出控制信号

    // 行为控制方法
    void chassis_behaviour_mode_set();  // 设置行为模式
    void chassis_behaviour_control_set(fp32 *vx_set_, fp32 *vy_set_, fp32 *angle_set);  // 行为控制设置

    // 具体行为控制方法
    void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set);
    void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
    void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    // 功能方法
    void chassis_gimbal_to_control_vector(fp32 *vx_set, fp32 *vy_set);  // 云台到控制向量转换
    void chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4]);  // 向量到麦轮速度转换
    fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);  // 电机编码到角度转换
};

// 全局底盘对象声明
extern Chassis chassis;

#endif