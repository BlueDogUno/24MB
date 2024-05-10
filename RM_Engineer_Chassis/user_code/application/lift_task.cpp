#include "lift_task.h"

#include "system_config.h" 

#include "lift.h"
#include "Communicate.h"
#include "auto.h"

/**
  * @brief          lift_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void lift_task(void *pvParameters)
{  

    //空闲一段时间
    vTaskDelay(LIFT_TASK_INIT_TIME);
    lift.init();
    // Auto.init();
    while(true){
      //设置模式
        lift.set_mode();

        //反馈数据
        lift.feedback_update();

        //设置控制量
        // Auto.motor_status_measure();
        // Auto.auto_control_set();
        lift.set_control();

        //解算
        lift.solve();

        //电流输出
        lift.output();

        lift.pump_contorl_send();

        //系统延时
        vTaskDelay(LIFT_TASK_INIT_TIME);

    }
        
}
