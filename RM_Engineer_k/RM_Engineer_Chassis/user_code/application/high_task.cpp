#include "high_task.h"

#include "system_config.h" 

#include "high.h"
#include "Communicate.h"
#include "auto.h"

/**
  * @brief          high_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void high_task(void *pvParameters) 
{
    //空闲一段时间
    vTaskDelay(HIGH_TASK_INIT_TIME);
    high.init();
    high.save_init();
    Auto.init();
    while(true) 
    {
        //设置模式
        high.set_mode();

        //反馈数据
        high.feedback_update();

        //设置控制量
        Auto.motor_status_measure();
        Auto.auto_control_set();
        high.set_control();

        //解算
        high.solve();

        //电流输出
        high.output();

        high.save_rc_control();

        high.save_control_send();

        high.sucker_contorl_send();

        //系统延时
        vTaskDelay(HIGH_TASK_INIT_TIME);
    }
}

