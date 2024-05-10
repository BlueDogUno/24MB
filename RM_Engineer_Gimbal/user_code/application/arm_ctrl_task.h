#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "cmsis_os.h"
#include "main.h"


//任务开始空闲一段时间
#define ARM_TASK_INIT_TIME 30



/**
  * @brief           arm_ctrl_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void arm_ctrl_task(void *pvParameters);



#endif
