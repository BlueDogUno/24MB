#ifndef MOTOR_H
#define MOTOR_H

#include "Pid.h"
#include "Can_receive.h"


//关节电机结构体 包含M3508 和M2006电机的角度控制
class Joint_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 angle_error; //状态保存变量

    fp32 angle_set; //rad
    fp32 total_angle;
    fp32 speed;
    fp32 speed_set;
    fp32 current_set;
    int16_t current_give;

    fp32 max_speed;
    fp32 min_speed; 
    
    fp32 max_angle;
    fp32 min_angle;
    void init(const motor_measure_t *motor_measure_);
};

#endif
