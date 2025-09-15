#include "communicate.h"
#include "main.h"
#include "string.h"
#include "bsp_usart.h"
#include "Can_receive.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#include "Chassis.h"

extern Can_receive can_receive;
Communicate communicate;

void Communicate::init()
{
    can_receive.init();
}


#ifdef __cplusplus
extern "C"
{
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        
        if (hfdcan == &CHASSIS_CAN) // 底盘CAN信息
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.Identifier)
            {
            // 底盘动力电机
            case CAN_MOTIVE_FR_MOTOR_ID:
                can_receive.get_motive_motor_measure(MOTIVE_FR_MOTOR, rx_data);
                break;
            case CAN_MOTIVE_FL_MOTOR_ID:
                can_receive.get_motive_motor_measure(MOTIVE_FL_MOTOR, rx_data);
                break;
            case CAN_MOTIVE_BL_MOTOR_ID:
                can_receive.get_motive_motor_measure(MOTIVE_BL_MOTOR, rx_data);
                break;
            case CAN_MOTIVE_BR_MOTOR_ID:
                can_receive.get_motive_motor_measure(MOTIVE_BR_MOTOR, rx_data);
                break;
            default:
                break;
            }
        }
        else if (hfdcan == &BOARD_COM_CAN) // 板间通信CAN
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.Identifier)
            {
            case CAN_GIMBAL_BOARD_COM_ID:
                can_receive.receive_gimbal_board_com(rx_data);
                break;
            default:
                break;
            }
        }
    }
}
#endif