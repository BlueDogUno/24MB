#ifndef LIFT_TASK_H
#define LIFT_TASK_H

#include "cmsis_os.h"
#include "main.h"


//任务开始空闲一段时间
#define LIFT_TASK_INIT_TIME 30



/**
  * @brief          lift_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void lift_task(void *pvParameters);



#endif
