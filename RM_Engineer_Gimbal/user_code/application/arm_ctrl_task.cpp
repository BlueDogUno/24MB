#include "arm_ctrl_task.h"

#include "system_config.h" 

#include "arm_ctrl.h"
#include "auto.h"
#include "Communicate.h"

/**
  * @brief          foreaarm_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void arm_ctrl_task(void *pvParameters) 
{
    //空闲一段时间
    vTaskDelay(ARM_TASK_INIT_TIME);
    arm_ctrl.init();
    Auto.init();

    while(true) 
    { 
        //设置模式
        arm_ctrl.set_mode();
        //反馈数据
        arm_ctrl.feedback_update();
        //设置控制量
        Auto.motor_status_measure();
        Auto.auto_control_set();
        arm_ctrl.set_control();
        //解算
        arm_ctrl.solve();
        //电流输出
        arm_ctrl.output();
        //系统延时
        vTaskDelay(ARM_TASK_INIT_TIME);
    }
}
