#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "struct_typedef.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_motive_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_motive_motor[num].last_ecd = chassis_motive_motor[num].ecd;
    chassis_motive_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_motive_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_motive_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_motive_motor[num].temperate = data[6];
}

/**
 * @brief 发送电机控制电流(0x201,0x202,0x203,0x204)
 */
void Can_receive::can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message.Identifier = CAN_CHASSIS_MOTIVE_ALL_ID;
    chassis_tx_message.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data);
}

/**
 * @brief 发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 */
void Can_receive::can_cmd_chassis_motive_motor_reset_ID(void)
{
    chassis_tx_message.Identifier = 0x700;
    chassis_tx_message.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data);
}

const motor_measure_t *Can_receive::get_chassis_motive_motor_measure_point(uint8_t i)
{
    return &chassis_motive_motor[i];
}

void Can_receive::receive_rc_board_com(uint8_t data[8])
{
    chassis_receive.ch_0 = (int16_t)(data[0] << 8 | data[1]);
    chassis_receive.ch_2 = (int16_t)(data[2] << 8 | data[3]);
    chassis_receive.ch_3 = (int16_t)(data[4] << 8 | data[5]);
    chassis_receive.v = (uint16_t)(data[6] << 8 | data[7]);
}

void Can_receive::receive_gimbal_board_com(uint8_t data[8])
{
    chassis_receive.s1 = data[0];
    chassis_receive.gimbal_behaviour = data[1];
    chassis_receive.gimbal_yaw_angle = (fp32)(int32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000;
}