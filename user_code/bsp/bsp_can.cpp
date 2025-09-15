#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void can_filter_init(void)
{
    
    // 配置FDCAN1滤波器 - 接受所有消息
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID;        // 标准ID
    can_filter_st.FilterIndex = 0;                   // 滤波器索引
    can_filter_st.FilterType = FDCAN_FILTER_MASK;    // 掩码模式
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 滤波到RXFIFO0
    can_filter_st.FilterID1 = 0x0000;                // 滤波器ID
    can_filter_st.FilterID2 = 0x0000;                // 滤波器掩码
    
    // 配置FDCAN1滤波器
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st);
    
    // 启动FDCAN1
    HAL_FDCAN_Start(&hfdcan1);
    // 使能FDCAN1接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    // 配置FDCAN2滤波器 - 使用不同的滤波器索引
    can_filter_st.FilterIndex = 14;                  // 使用不同的滤波器索引
    
    // 配置FDCAN2滤波器
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st);
  
    
    // 启动FDCAN2
    HAL_FDCAN_Start(&hfdcan2);
    
    // 使能FDCAN2接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

}