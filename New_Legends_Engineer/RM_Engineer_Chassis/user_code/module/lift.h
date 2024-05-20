#ifndef LIFT_H
#define LIFT_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "tim.h"
#include "auto.h"

#define LIFT_DEBUG_FLAG 0

//拨矿电机方向
#define LIFT_UPLOAD_MOTOR_TURN 1
//伸爪电机方向
#define LIFT_STRETCH_MOTOR_TURN 1

//任务控制间隔 2ms
#define LIFT_CONTROL_TIME_MS 2

//控制变换的遥控器通道
#define LIFT_X_CHANNEL 3

#define LIFT_OPEN_RC_SCALE 200.0f // 遥控器乘以该比例发送到can上

//选择取矿机构状态 开关通道号
#define LIFT_MODE_CHANNEL 0
#define LEFT_CHANNEL 0
#define RIGHT_CHANNEL 1

//误差区间
#define ANGLE_ERR_TOLERANT 2.0f

//抬升电机ID
#define CAN_LIFT_L_MOTOR 0
#define CAN_LIFT_R_MOTOR 1

//23抬不起来的东西
//抬升前电机速度环PID                           //主要是晃动明显
#define LIFT_F_MOTOR_SPEED_PID_KP 1500.0f   //500.0f
#define LIFT_F_MOTOR_SPEED_PID_KI 200.0f    //0.0f
#define LIFT_F_MOTOR_SPEED_PID_KD 0.0f
#define LIFT_F_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
#define LIFT_F_MOTOR_SPEED_PID_MAX_OUT 12000.0f

//抬升前电机角度环PID
#define MOTIVE_MOTOR_ANGLE_PID_KP 1.0f 
#define MOTIVE_MOTOR_ANGLE_PID_KI 0.0f
#define MOTIVE_MOTOR_ANGLE_PID_KD 0.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_OUT 100.0f

//抬升后电机速度环PID                           //这边阻力很大
// #define LIFT_B_MOTOR_SPEED_PID_KP 1500.0f
// #define LIFT_B_MOTOR_SPEED_PID_KI 300.0f
// #define LIFT_B_MOTOR_SPEED_PID_KD 0.0f
// #define LIFT_B_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
// #define LIFT_B_MOTOR_SPEED_PID_MAX_OUT 12000.0f

// //抬升后电机角度环PID
// #define LIFT_B_MOTOR_ANGLE_PID_KP 1.0f 
// #define LIFT_B_MOTOR_ANGLE_PID_KI 0.0f
// #define LIFT_B_MOTOR_ANGLE_PID_KD 0.0f
// #define LIFT_B_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
// #define LIFT_B_MOTOR_ANGLE_PID_MAX_OUT 100.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_motor_RPM_TO_VECTOR 0.000415809748903494517209f
#define LIFT_MOTOR_RPM_TO_VECTOR_SEN M3508_motor_RPM_TO_VECTOR
#define LIFT_MOTOR_RPM_TO_VECTOR_SEN M3508_motor_RPM_TO_VECTOR

// 各电机角度限幅
#define LIFT_LIMIT_ANGLE 20.0f
#define LIFT_MIN_ANGLE -20.0f
#define LIFT_MAX_ANGLE 20.0f
#define MOTOR_SPEED_TO_LIFT_SPEED 0.25f

#define LIFT_INIT_LEN 75.0f
#define LIFT_SKY_LEN (LIFT_STANDARD_LEN + 200.0f)
#define LIFT_STANDARD_LEN 530.0f
#define LIFT_GROUND_LEN 0.0f
#define LIFT_DELIVERY_LEN 0.0f

//抬升过程最大速度
#define NORMAL_MAX_LIFT_SPEED 2.0f //2.0
//伸爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 2.0f //2.0

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0

#define left_rocker_right           (lift_RC->rc.ch[2] > 0)
#define left_rocker_left            (lift_RC->rc.ch[2] < 0)
#define left_rocker_mid             (lift_RC->rc.ch[2] == 0)

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
              

// 用于自动模式下的电机控制
typedef enum
{
    SPIN_MOTOR = 0,
    YAW_MOTOR,
    SUCTION_MOTOR,
    MOTOR_NUM,
} auto_motor_state;  

typedef enum{
    ZROE_FORCE=0,
    close = 0,
    open,
} motor_state;

typedef enum
{
    LIFT_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    LIFT_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

    LIFT_CLOSE,                       //全自动，操作手没有权限控制

    LIFT_HOLD,

} lift_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    LIFT_AUTO,

    LIFT_HAND,      //用了自动模式的都说好

    LIFT_HOLDM,

} lift_mode_e;      //控制模式


class Lift 
{
public:
    const RC_ctrl_t *lift_RC; //抬升机构使用的遥控器指针
    RC_ctrl_t *last_lift_RC; //抬升机构使用的遥控器指针

    uint16_t lift_last_key_v;  //遥控器上次按键

    lift_behaviour_e lift_behaviour_mode; //抬升机构行为状态机
    lift_behaviour_e last_lift_behaviour_mode; //抬升机构上次行为状态机

    lift_mode_e lift_mode; //抬升机构控制状态机
    lift_mode_e last_lift_mode; //抬升机构上次控制状态机

    M3508_motor lift_motive_motor[2];
    fp32 motor_start_angle[2];
    bool_t lift_state;

    fp32 angle_hold_set;

    uint8_t motor_status[2];
    
    //任务流程
    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();
    
    void solve();

    void output();

    //行为控制
    void behaviour_control_set(fp32 *vz_set);

    void lift_open_set_control(fp32 *vz_set);

    void motor_set_control(M3508_motor *motor);    
    
    void auto_control(auto_mode_e *auto_mode);

    //功能函数
    void pump_contorl_send();

    void motor_angle_limit(M3508_motor *motor);

};


extern Lift lift;
extern uint8_t pump_state;



#endif
