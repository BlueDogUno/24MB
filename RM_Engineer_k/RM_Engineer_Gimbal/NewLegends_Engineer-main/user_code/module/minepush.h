#ifndef MINEPUSH_H
#define MINEPUSH_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "auto.h"

//拨矿电机方向
#define MINE_UPLOAD_MOTOR_TURN 1
//伸爪电机方向
#define MINE_STRETCH_MOTOR_TURN 1

//任务控制间隔 2ms
#define MINE_CONTROL_TIME_MS 2

//前后的遥控器通道号码
#define MINE_X_CHANNEL 2
//左右的遥控器通道号码
#define MINE_Y_CHANNEL 1

#define MINE_OPEN_RC_SCALE 300 // 遥控器乘以该比例发送到can上

//选择取矿机构状态 开关通道号
#define MINE_MODE_CHANNEL 1
//选择取矿机构状态 开关通道号
#define STRETCH_MODE_CHANNEL 0
//选择图传舵机状态 开关通道号 
#define PHOTO_MODE_CHANNEL 1        //左拨杆

//拨矿电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 100.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 200.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 0.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 5000.0f

//拨矿电机角度环PID
#define MOTIVE_MOTOR_ANGLE_PID_KP 2.0f 
#define MOTIVE_MOTOR_ANGLE_PID_KI 0.0f
#define MOTIVE_MOTOR_ANGLE_PID_KD 2000.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_OUT 2000.0f


//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MOTOR_SPEED_TO_MINE_SPEED 0.25f

// 伸爪电机角度限幅
#define STRENTCH_LIMIT_ANGLE 0.0f

//拨矿过程最大速度
#define NORMAL_MAX_MINE_SPEED 4.0f //2.0
//伸爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 4.0f //2.0
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0

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

typedef enum
{
    MINE_PUSH_LEFT_ID,
    MINE_PUSH_RIGHT_ID,
    MINE_STRETCH_L_ID,
    MINE_STRETCH_R_ID,
    
};                      

typedef enum
{
    MINE_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    MINE_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

    MINE_CLOSE,                       //全自动，操作手没有权限控制

} mine_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    MINE_AUTO,      //无敌的自动模式

    MINE_HAND,      //用了自动模式的都说好

} mine_mode_e;      //控制模式


class MinePush {
public:
    const RC_ctrl_t *mine_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_mine_RC; //底盘使用的遥控器指针

    uint16_t mine_last_key_v;  //遥控器上次按键

    mine_behaviour_e mine_behaviour_mode; //底盘行为状态机
    mine_behaviour_e last_mine_behaviour_mode; //底盘上次行为状态机

    mine_mode_e mine_mode; //底盘控制状态机
    mine_mode_e last_mine_mode; //底盘上次控制状态机

    Mine_motor mine_motive_motor[4];
    fp32 stretch_moto_start_angle[4];

    int8_t motor_status[2];

    uint8_t photo_flag_pitch; 

    
    

    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();

    //行为模式
    void behaviour_control_set(fp32 *vx_set, fp32 *vy_set);

    void mine_open_set_control(fp32 *vx_set, fp32 *vy_set);

    void motor_set_control(Mine_motor *motor);

    void solve();

    void output();

    void auto_control(auto_mode_e *auto_mode);

};



typedef enum
{
    WALK_MODE = 0,      //底盘运动时
    EXCHANGE_MODE,      //兑矿时

    LOOK_UP=0,            //抬头
    LOOK_MID,           //平视
    LOOK_DOWN,          //俯视

} photospin_state;      //控制模式

#define yaw_walk_data 1500               //底盘运动时
#define yaw_exchange_data 2450          //兑矿时

#define pitch_look_up_data 900          //抬头
#define pitch_look_mid_data 725        //平视
#define pitch_look_down_data  530      //俯视

#define right_rocker_up              (mine_RC->rc.ch[1] > 0)
#define right_rocker_down            (mine_RC->rc.ch[1] < 0)
#define right_rocker_mid             (mine_RC->rc.ch[1] == 0)

#define left_rocker_right           (mine_RC->rc.ch[2] > 0)
#define left_rocker_left            (mine_RC->rc.ch[2] < 0)
#define left_rocker_mid             (mine_RC->rc.ch[2] == 0)

class PhotoSpin{//图传舵机数据结构体
    public:

        bool_t yaw_state;               //图传舵机yaw轴状态

        uint8_t pitch_state;            //图传舵机pitch轴状态

        void output();                  //控制图传舵机

        uint8_t yaw_flag;

        uint8_t pitch_flag;

        bool_t sucker_flag;
};

extern PhotoSpin photospin;

extern MinePush minepush;


#endif