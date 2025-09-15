#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define CHASSIS_CAN hfdcan2
#define BOARD_COM_CAN hfdcan1

// 底盘动力电机编号
enum motive_chassis_motor_id_e
{
  // 底盘动力电机接收
  MOTIVE_FR_MOTOR = 0,
  MOTIVE_FL_MOTOR,
  MOTIVE_BL_MOTOR,
  MOTIVE_BR_MOTOR,
};

/* CAN send and receive ID */
typedef enum
{
  // 底盘动力电机接收ID FDCAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  // 板间通信ID
  CAN_RC_BOARM_COM_ID = 0x301,
  CAN_GIMBAL_BOARD_COM_ID = 0x302,

} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  uint16_t last_ecd;
} motor_measure_t;

// 底盘接收数据结构体
typedef struct
{
    // 云台状态
    uint8_t s1;
    uint8_t gimbal_behaviour;
    fp32 gimbal_yaw_angle;
    
    // 云台控制模式
    uint8_t gimbal_control_mode;      // 云台控制底盘的模式
    uint8_t last_gimbal_control_mode; // 上一次的控制模式
    
    // 云台转向标志
    uint8_t gimbal_turn_flag;         // 云台转向标志
    uint8_t last_gimbal_turn_flag;    // 上一次的转向标志
    
    // 云台发送的速度控制指令
    fp32 vx;                          // 纵向速度指令
    fp32 vy;                          // 横向速度指令
    fp32 wz;                          // 旋转速度指令
    
    // 云台发送的其他控制信息
    uint8_t ui_update_flag;           // UI更新标志
    uint8_t chassis_control_enable;   // 底盘控制使能标志
    
} chassis_receive_t;


class Can_receive
{
public:
  // 动力电机反馈数据结构体
  motor_measure_t chassis_motive_motor[4];

  // 发送数据结构体
  FDCAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  // 板间通信
  // 底盘接收信息
  chassis_receive_t chassis_receive;

  void init();

  // 电机数据接收
  void get_motive_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); // 动力电机数据

  void can_cmd_chassis_motive_motor_reset_ID();

  const motor_measure_t *get_chassis_motive_motor_measure_point(uint8_t i);

  // 板间通信函数

  void receive_gimbal_board_com(uint8_t data[8]);

};

#endif