#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#include "CAN_receive.h"
#include "remote_control.h"

/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 1

    
void run();
#endif