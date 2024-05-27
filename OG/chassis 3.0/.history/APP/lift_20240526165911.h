/*
    抬升任务
    使用函数指针方式书写
    宁波工程学院 机器人学院 电气201张超 于RMUC2022赛季书写
*/

#ifndef LIFT_H
#define LIFT_H

#include "remote_control.h"
#include "can.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
// #include "catch_auto.h"

#define LIFT_HOLD 

//大结构体
typedef struct
{

    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[6];
    // const auto_t *auto_behave;
    const reset_t *reset_key;
    
    //气泵控制
    uint8_t pump_state;     //气泵状态
  

    //函数指针定义
    void (*init)();
    void (*pid_init)();
    void (*lift_set_mode)();
    void (*lift_control)();  
    void (*ore_set_mode)();
    void (*ore_control)(); 
    fp32 (*PID_calc)();

    void (*can_send)();
    void (*sensor)();

    //电机电流、电机状态存储
    struct
    {   
        struct
        {
            int16_t left;
            int16_t right;
            int16_t left_target;
            int16_t right_target;
            int16_t left_speed;
            int16_t right_speed;
            int     state;
        }lift;
        
        struct
        {
            int16_t left;
            int16_t right;
            int16_t left_target;
            int16_t right_target;
            int16_t left_speed;
            int16_t right_speed;
            int     state;
        }ore;

    }can;

    struct
    {
        
        int8_t    photogate_1;
        int8_t    photogate_2;

    }sensor_data;
    
    float lift_lenth;

    //遥控器状态命名
    #define left_switch_is_up           (strt.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (strt.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (strt.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (strt.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (strt.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (strt.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (strt.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (strt.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (strt.rc_data->rc.ch[3] == 0)

    #define dial_forward                (strt.rc_data->rc.ch[4] > 0)
    #define dial_reserve                (strt.rc_data->rc.ch[4] < 0)
    #define dial_stop                   (strt.rc_data->rc.ch[4] == 0)

    //电机状态命名
    #define state_is_stop               (strt.can.lift.state == stop)
    #define state_is_up                 (strt.can.lift.state == up)
    #define state_is_down               (strt.can.lift.state == down)
    #define state_is_shut               (strt.can.lift.state == shut)
    
    #define ore_state_is_stop           (strt.can.ore.state == stop)
    #define ore_state_is_in             (strt.can.ore.state == in)
    #define ore_state_is_out            (strt.can.ore.state == out)

}strt_t;
// 电控限位值
float lift_down = -10.0f;
float lift_up = -520.0f;


int8_t ore_flag = 0;
int8_t ore_last_flag = 0;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

//PID算法结构体
typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} lift_pid_strt;


//一号电机PID
float LIFT_LEFT_KP     =   12.0f;
float LIFT_LEFT_KI     =   1.0f;
float LIFT_LEFT_KD     =   0.0f;
float LIFT_LEFT_MOUT   =   16000.0f;
float LIFT_LEFT_MIOUT  =   8000.0f;
//二号电机PID
float LIFT_RIGHT_KP     =   12.0f;
float LIFT_RIGHT_KI     =   1.0f;
float LIFT_RIGHT_KD     =   0.0f;
float LIFT_RIGHT_MOUT   =   16000.0f;
float LIFT_RIGHT_MIOUT  =   8000.0f;
//三号电机PID
float ORE_LEFT_KP     =   1.0f;
float ORE_LEFT_KI     =   0.0f;
float ORE_LEFT_KD     =   0.0f;
float ORE_LEFT_MOUT   =   16000.0f;
float ORE_LEFT_MIOUT  =   1.0f;
//四号电机PID
float ORE_RIGHT_KP     =   1.0f;
float ORE_RIGHT_KI     =   0.0f;
float ORE_RIGHT_KD     =   0.0f;
float ORE_RIGHT_MOUT   =   16000.0f;
float ORE_RIGHT_MIOUT  =   1.0f;    



enum
{
    motor_left_ID = 0x201,
    motor_right_ID = 0x202,
    ore_motor_left_ID = 0x203,
    ore_motor_right_ID = 0x204,
}motor_ID;

// 各种档位的赋值，方便模式设置
enum
{
    stop    =   0,
    up,
    down,
    in,
    out,
    shut,
    open = 1,
    close = 0
}state_type;

void lift_set_mode(void);
void ore_set_mode(void);
void lift_control(void);
void ore_control(void);
// void lift_auto_control(void);
void can_send(void);
void lift_PID_init(void);
fp32 lift_PID_calc(lift_pid_strt *pid, int16_t ref, int16_t set);
void lift_init(void);
void pump_contorl_send(void);   //气泵控制   

#endif

