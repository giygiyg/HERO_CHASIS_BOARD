#include "Chassis.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#ifdef __cplusplus
extern "C"
{
#include "user_lib.h"
}
#endif

// 底盘模块对象
Chassis chassis;

// 临时变量，用于调试
int debug_var = 0;
fp32 debug_vx, debug_vy;

/**
 * @brief          初始化变量，包括PID初始化，3508底盘电机指针初始化
 * @param[out]     none
 * @retval         none
 */
void Chassis::init()
{
    // 设置初始状态机
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    chassis_mode = CHASSIS_VECTOR_RAW;
    last_chassis_mode = chassis_mode;

    // 初始化底盘电机
    for (uint8_t i = 0; i < 4; ++i)
    {
        // 动力电机数据
        chassis_motive_motor[i].init(can_receive.get_chassis_motive_motor_measure_point(i));
        // 初始化PID
        fp32 motive_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, 
                                         MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, 
                                         MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        chassis_motive_motor[i].speed_pid.init(PID_SPEED, motive_speed_pid_parm, 
                                              &chassis_motive_motor[i].speed, 
                                              &chassis_motive_motor[i].speed_set, NULL);
        chassis_motive_motor[i].speed_pid.pid_clear();
    }

    // 一阶滤波参数初始化
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    // 初始化角度Z轴PID
    fp32 z_angle_pid_parm[5] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, 
                                CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 
                                CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT};
    chassis_wz_angle_pid.init(PID_ANGLE, z_angle_pid_parm, &chassis_relative_angle, 
                             &chassis_relative_angle_set, NULL);
    chassis_wz_angle_pid.pid_clear();

    // 速度限幅设置
    x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;

    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;

    z.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    z.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    // 更新数据
    feedback_update();
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     none
 * @retval         none
 */
void Chassis::set_mode()
{
    chassis_behaviour_mode_set();
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     none
 * @retval         none
 */
void Chassis::feedback_update()
{
    // 切入跟随云台模式
    if ((last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && 
        (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW))
    {
        chassis_relative_angle_set = INIT_YAW_SET;
    }
    // 切入不跟随云台模式
    else if ((last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && 
             (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW))
    {
        chassis_yaw_set = 0;
    }
    // 切入原始模式
    else if ((last_chassis_mode != CHASSIS_VECTOR_RAW) && 
             (chassis_mode == CHASSIS_VECTOR_RAW))
    {
        chassis_yaw_set = 0;
    }

    // 更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        // 更新动力电机速度，加速度是速度的PID微分
        chassis_motive_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * 
                                        chassis_motive_motor[i].motor_measure->speed_rpm;
        chassis_motive_motor[i].accel = chassis_motive_motor[i].speed_pid.data.error_delta * 
                                        CHASSIS_CONTROL_FREQUENCE;
    }

    // 更新底盘x, y, z速度值,右手坐标系
    x.speed = (-chassis_motive_motor[0].speed + chassis_motive_motor[1].speed + 
               chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * 
               MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    y.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed + 
               chassis_motive_motor[2].speed + chassis_motive_motor[3].speed) * 
               MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    z.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed - 
               chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * 
               MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    // 底盘相对于云台的角度,由云台发送过来 编码器中的角度
    chassis_relative_angle = can_receive.chassis_receive.gimbal_yaw_angle;

    
}

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]     none
 * @retval         none
 */
void Chassis::set_contorl()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

    // 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);
    
    // 跟随云台模式
    if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sin(-chassis_relative_angle);
        cos_yaw = cos(-chassis_relative_angle);

        // 根据云台转向标志确定速度方向
        if((can_receive.chassis_receive.gimbal_turn_flag == 1) || 
           ((can_receive.chassis_receive.last_gimbal_turn_flag == 0) && 
            (can_receive.chassis_receive.gimbal_turn_flag == 2)))
        {
            x.speed_set = -(cos_yaw * vx_set + sin_yaw * vy_set);
            y.speed_set = -(-sin_yaw * vx_set + cos_yaw * vy_set);
        }

        if((can_receive.chassis_receive.gimbal_turn_flag == 0) || 
           ((can_receive.chassis_receive.last_gimbal_turn_flag == 1) && 
            (can_receive.chassis_receive.gimbal_turn_flag == 2)))
        {
            x.speed_set = cos_yaw * vx_set + sin_yaw * vy_set;
            y.speed_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        }
        
        // 更新云台转向标志
        if(can_receive.chassis_receive.gimbal_turn_flag != 2)
        {
            can_receive.chassis_receive.last_gimbal_turn_flag = can_receive.chassis_receive.gimbal_turn_flag;
        }
        
        // 设置控制相对云台角度
        chassis_relative_angle_set = rad_format(angle_set);

        // 速度限幅
        x.speed_set = fp32_constrain(x.speed_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(y.speed_set, y.min_speed, y.max_speed);
        z.speed_set = fp32_constrain(z.speed_set, z.min_speed, z.max_speed);
    }
    // 不跟随云台模式
    else if (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        // "angle_set" 是旋转速度控制
        z.speed_set = angle_set;
        // 速度限幅
        x.speed_set = fp32_constrain(vx_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
    }
    // 原始模式
    else if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        // 在原始模式，设置值是直接发送到CAN总线
        x.speed_set = vx_set;
        y.speed_set = vy_set;
        z.speed_set = angle_set;
        chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
 * @brief          解算数据,并进行PID计算
 * @param[out]     none
 * @retval         none
 */
void Chassis::solve()
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 动力电机目标速度

    uint8_t i = 0;

    // 麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(wheel_speed);

    // 原始模式直接设置电流值
    if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].current_give = (int16_t)(wheel_speed[i]);
        }
        // raw控制直接返回
        return;
    }

    // 计算动力电机控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_motive_motor[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    // 速度限制
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].speed_set *= vector_rate;
        }
    }

    // 计算PID
    for (i = 0; i < 4; i++)
    {
        // 计算动力电机的输出电流
        chassis_motive_motor[i].current_set = chassis_motive_motor[i].speed_pid.pid_calc();
    }
    
    // 调试变量
    debug_vx = chassis_motive_motor[0].current_set;
}

/**
 * @brief          输出电流到电机
 * @param[in]      none
 * @retval         none
 */
void Chassis::output()
{
    // 赋值电流值
    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].current_give = (int16_t)(chassis_motive_motor[i].current_set);
        // 无力模式下输出零电流
        if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
        {
            chassis_motive_motor[i].current_give = 0;
        }
    }

    // 电流输出控制,通过调整宏定义控制
    for (int i = 0; i < 4; i++)
    {
#if CHASSIS_MOTIVE_MOTOR_NO_CURRENT
        chassis_motive_motor[i].current_give = 0;
#endif
    }

    // 发送电流值到CAN总线
    can_receive.can_cmd_chassis_motive_motor(chassis_motive_motor[0].current_give, 
                                            chassis_motive_motor[1].current_give,
                                            chassis_motive_motor[2].current_give, 
                                            chassis_motive_motor[3].current_give);
                                            
    // 调试变量
    debug_vx = chassis_motive_motor[0].current_give;
}

/**
 * @brief          通过逻辑判断，设置底盘行为模式
 * @param[in]      none
 * @retval         none
 */
void Chassis::chassis_behaviour_mode_set()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    last_chassis_mode = chassis_mode;
    
    // 云台控制模式设置
    if (can_receive.chassis_receive.gimbal_control_mode == GIMBAL_CONTROL_FOLLOW) // 跟随云台模式
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (can_receive.chassis_receive.gimbal_control_mode == GIMBAL_CONTROL_NO_FOLLOW) // 自主运动模式
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (can_receive.chassis_receive.gimbal_control_mode == GIMBAL_CONTROL_ZERO_FORCE) // 无力模式
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    // 根据行为模式选择底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) // 无力模式
    {
        chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) // 跟随云台模式
    {
        chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) // 自主运动模式
    {
        chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动
 * @param[out]     vx_set, 通常控制纵向移动
 * @param[out]     vy_set, 通常控制横向移动
 * @param[out]     wz_set, 通常控制旋转运动
 * @param[in]      包括底盘所有信息
 * @retval         none
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN) // 底盘控制测试用
    {
        chassis_open_set_control(vx_set, vy_set, angle_set);
    }
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，设定值都设置为0
 * @param[in]      vx_set 前进的速度设定值
 * @param[in]      vy_set 左右的速度设定值
 * @param[in]      wz_set 旋转的速度设定值
 * @retval         none
 */
void Chassis::chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，设定值都设置为0
 * @param[in]      vx_set 前进的速度设定值
 * @param[in]      vy_set 左右的速度设定值
 * @param[in]      wz_set 旋转的速度设定值
 * @retval         none
 */
void Chassis::chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度
 * @param[in]      vx_set 前进的速度设定值
 * @param[in]      vy_set 左右的速度设定值
 * @param[in]      angle_set 底盘与云台控制到的相对角度
 * @retval         none
 */
void Chassis::chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    // 从云台接收数据得出速度设定值
    chassis_gimbal_to_control_vector(vx_set, vy_set);
    
    // 设置角度设定值为0（去除扭腰功能）
    *angle_set = 0.0f;
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘旋转速度由参数直接设定
 * @param[in]      vx_set 前进的速度设定值
 * @param[in]      vy_set 左右的速度设定值
 * @param[in]      wz_set 底盘设置的旋转速度
 * @retval         none
 */
void Chassis::chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_gimbal_to_control_vector(vx_set, vy_set);
    // 设置旋转速度为0（去除小陀螺功能）
    *wz_set = 0.0f;
}

/**
 * @brief          底盘开环的行为状态机下，设定值会直接发送到can总线上
 * @param[in]      vx_set 前进的速度设定值
 * @param[in]      vy_set 左右的速度设定值
 * @param[in]      wz_set 旋转速度设定值
 * @retval         none
 */
void Chassis::chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    // 从云台接收开环控制数据
    *vx_set = can_receive.chassis_receive.vx * CHASSIS_OPEN_GIMBAL_SCALE;
    *vy_set = -can_receive.chassis_receive.vy * CHASSIS_OPEN_GIMBAL_SCALE;
    *wz_set = -can_receive.chassis_receive.wz * CHASSIS_OPEN_GIMBAL_SCALE;
}

/**
 * @brief          根据云台数据，计算纵向和横移速度
 * @param[out]     vx_set 纵向速度指针
 * @param[out]     vy_set 横向速度指针
 * @retval         none
 */
void Chassis::chassis_gimbal_to_control_vector(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    fp32 vx_set_channel, vy_set_channel;
    
    // 直接从云台接收数据
    vx_set_channel = can_receive.chassis_receive.vx * CHASSIS_VX_GIMBAL_SEN;
    vy_set_channel = can_receive.chassis_receive.vy * -CHASSIS_VY_GIMBAL_SEN;

    // 一阶低通滤波代替斜波作为底盘速度输入
    chassis_cmd_slow_set_vx.first_order_filter_cali(vx_set_channel);
    chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);

    // 停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_GIMBAL_DEADLINE * CHASSIS_VX_GIMBAL_SEN && 
        vx_set_channel > -CHASSIS_GIMBAL_DEADLINE * CHASSIS_VX_GIMBAL_SEN)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_GIMBAL_DEADLINE * CHASSIS_VY_GIMBAL_SEN && 
        vy_set_channel > -CHASSIS_GIMBAL_DEADLINE * CHASSIS_VY_GIMBAL_SEN)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set 纵向速度
 * @param[in]      vy_set 横向速度
 * @param[in]      wz_set 旋转速度
 * @param[out]     wheel_speed 四个麦轮速度
 * @retval         none
 */
void Chassis::chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4])
{
    // 麦轮运动学解算
    wheel_speed[0] = -x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[1] = x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[2] = x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[3] = -x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd 电机当前编码
 * @param[in]      offset_ecd 电机中值编码
 * @retval         相对角度，单位rad
 */
fp32 Chassis::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}