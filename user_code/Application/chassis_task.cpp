#include "chassis_task.h"
#include "gpio.h"
#include "system_config.h" 
#include "Communicate.h"
#include "chassis.h"



uint8_t chassis_flag = 0;

/**
  * @brief          chassis_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void chassis_task(void *pvParameters)
{

    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis.init();
    communicate.init();
    // }

    while(true)
    {

      chassis_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

      //设置模式
      chassis.set_mode();
      //反馈数据
      chassis.feedback_update();
      //设置控制量
      chassis.set_contorl();
      //解算
      chassis.solve();

      //电流输出
      chassis.output();


      //系统延时
      vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
