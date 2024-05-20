/*      宁波工程学院    电气201         张超
        2021.12.6      工程机器�?      夹爪
        使用函数指针及模块化编程
        
        1.大结构体，用于存储除枚举类型数据，以便其他文件读�?
        2.框架函数，仿照官方C型开发板步兵代码编写，以便后期添加功能及更改
        3.功能函数，由基本功能函数修改而来，完成串口收发、PID算法等工�?
        
        写代码别这样写，函数指针巨垃�?
*/
#ifndef CATCH_H
#define CATCH_H

//头文件引�?
#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"
#include "CAN_receive.h"
#include "gpio.h"
#include "tim.h"
#include "catch_auto.h"
//大结构体
typedef struct
{
    //遥控器指�?
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[4];
    const sensor_measure_t *sensor_measure[2];
    const auto_t *auto_behave;
    const reset_t *reset_key;
    //函数指针定义

    //电机电流、电机状态存�?
    struct
    {
        int16_t left;   //翻转�?
        int16_t right;  //翻转�?
        int16_t stretch;//出矿
        int16_t catch;  //夹紧
        int16_t left_speed_target;   
        int16_t right_speed_target;
        int16_t left_target;   
        int16_t right_target;   
        int16_t stretch_target;
        fp32    stretch_lenth;
        int16_t catch_target;
        int16_t left_speed;   
        int16_t right_speed;  
        int16_t stretch_speed;
        int16_t catch_speed;  
        int8_t     flip_state;
        int8_t     last_flip_state;
        int8_t     stretch_state;
        int8_t     catch_state;
    }can;
    int8_t catch_sensor;//光电�?

    float flip_angle;
    float flip_stay_angle;
    float stretch_lenth;

    float flip_reset_angle;
    int16_t reset_flag;
    int64_t reset_last_flag;

    //遥控器状态命�?
    #define left_switch_is_up           (catch.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (catch.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (catch.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (catch.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (catch.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (catch.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (catch.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (catch.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (catch.rc_data->rc.ch[3] == 0)


    //电机状态命�?
    #define flip_state_is_stop               (catch.can.flip_state == stop)
    #define flip_state_is_forward            (catch.can.flip_state == forward)
    #define flip_state_is_reverse            (catch.can.flip_state == reverse)

    #define stretch_state_is_stop       (catch.can.stretch_state == stop)
    #define stretch_state_is_out        (catch.can.stretch_state == stretch_out)
    #define stretch_state_is_back       (catch.can.stretch_state == stretch_back)

    #define catch_state_is_stop         (catch.can.catch_state == stop)
    #define catch_state_is_close        (catch.can.catch_state == close)
    #define catch_state_is_open         (catch.can.catch_state == open)
    #define catch_state_is_shut         (catch.can.catch_state == shut)

    #define catch_sensor_0              (catch.catch_sensor == 0)
    #define catch_sensor_1              (catch.catch_sensor == 1)
}catch_ctrl_t;


fp32 flip_angle_up = 250.0f;
fp32 flip_angle_down = -5.0f;
fp32 stretch_out_lenth = 1050.0f;
fp32 stretch_back_lenth = 70.0f;



enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum
{
    stop    =   0,
    forward,
    reverse,
    stretch_out,
    stretch_back,
    close,
    open,
    shut
}catch_state;




typedef struct
{
    uint8_t mode;
    //PID 三参�?
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输�?
    fp32 max_iout; //最大积分输�?

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分�? 0最�? 1上一�? 2上上�?
    fp32 error[3]; //误差�? 0最�? 1上一�? 2上上�?

} pid_strt;


//一号电机PID
float CATCH_LEFT_KP     =   20.0f;
float CATCH_LEFT_KI     =   0.0f;
float CATCH_LEFT_KD     =   200.0f;
float CATCH_LEFT_MOUT   =   16000.0f;
float CATCH_LEFT_MIOUT  =   1.0f;
//二号电机PID
float CATCH_RIGHT_KP     =   20.0f;
float CATCH_RIGHT_KI     =   0.0f;
float CATCH_RIGHT_KD     =   200.0f;
float CATCH_RIGHT_MOUT   =   16000.0f;
float CATCH_RIGHT_MIOUT  =   1.0f;
//三号电机PID
float CATCH_STRETCH_KP     =   10.0f;
float CATCH_STRETCH_KI     =   0.0f;
float CATCH_STRETCH_KD     =   0.0f;
float CATCH_STRETCH_MOUT   =   2000.0f;
float CATCH_STRETCH_MIOUT  =   1.0f;
//四号电机PID
float CATCH_CATCH_KP     =   10.0f;
float CATCH_CATCH_KI     =   0.0f;
float CATCH_CATCH_KD     =   0.0f;
float CATCH_CATCH_MOUT   =   4000.0f;
float CATCH_CATCH_MIOUT  =   1.0f;    
//翻爪角度�?
float CATCH_ANGLE_KP     =   60.0f;
float CATCH_ANGLE_KI     =   0.0f;
float CATCH_ANGLE_KD     =   5.0f;
float CATCH_ANGLE_MOUT   =   400.0f;
float CATCH_ANGLE_MIOUT  =   1.0f;
//自动模式


void catch_set_mode(void);
void catch_control(void);
void catch_can_send(void);
void catch_sensor(void);
void oreflip_servo(void);
fp32 catch_PID_calc(pid_strt *pid, int16_t ref, int16_t set);
void electromagnet_control(void);
void catch_PID_init(void);
void catch_init(void);
void auto_ore(void);

#endif
