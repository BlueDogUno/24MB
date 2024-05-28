#ifndef CATCH_H
#define CATCH_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "auto.h"

#define CATCH_DEBUG_FLAG 0

//拨矿电机方向
#define CATCH_UPLOAD_MOTOR_TURN 1
//伸爪电机方向
#define CATCH_STRETCH_MOTOR_TURN 1

//任务控制间隔 2ms
#define CATCH_CONTROL_TIME_MS 2

//前后的遥控器通道号码
#define CATCH_X_CHANNEL 1 //pitch flip
//左右的遥控器通道号码
#define CATCH_Y_CHANNEL 0 //yaw
//
#define CATCH_Z_CHANNEL 2 //roll

#define CATCH_OPEN_RC_SCALE 300 // rz遥控器除以该比例发送到can上

//选择取矿机构状态 开关通道号
#define CATCH_MODE_CHANNEL 1

//拨矿电机速度环PID     翻爪flip左    需要的力有些大
#define FLIP_L_MOTOR_SPEED_PID_KP 1000.0f
#define FLIP_L_MOTOR_SPEED_PID_KI 00.0f
#define FLIP_L_MOTOR_SPEED_PID_KD 2000.0f
#define FLIP_L_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
#define FLIP_L_MOTOR_SPEED_PID_MAX_OUT 12000.0f

//拨矿电机角度环PID
#define FLIP_L_MOTOR_ANGLE_PID_KP 3.0f 
#define FLIP_L_MOTOR_ANGLE_PID_KI 0.0f
#define FLIP_L_MOTOR_ANGLE_PID_KD 0.0f
#define FLIP_L_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define FLIP_L_MOTOR_ANGLE_PID_MAX_OUT 11000.0f

//拨矿电机速度环PID     翻爪flip右    需要的力有些大
#define FLIP_R_MOTOR_SPEED_PID_KP 1000.0f
#define FLIP_R_MOTOR_SPEED_PID_KI 0.0f
#define FLIP_R_MOTOR_SPEED_PID_KD 2000.0f
#define FLIP_R_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
#define FLIP_R_MOTOR_SPEED_PID_MAX_OUT 12000.0f

//拨矿电机角度环PID
#define FLIP_R_MOTOR_ANGLE_PID_KP 3.0f 
#define FLIP_R_MOTOR_ANGLE_PID_KI 0.0f
#define FLIP_R_MOTOR_ANGLE_PID_KD 0.0f
#define FLIP_R_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define FLIP_R_MOTOR_ANGLE_PID_MAX_OUT 11000.0f


//拨矿电机速度环PID         yaw轴       操作延迟
#define YAW_MOTOR_SPEED_PID_KP 1000.0f
#define YAW_MOTOR_SPEED_PID_KI 0.0f
#define YAW_MOTOR_SPEED_PID_KD 2000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 10000.0f

//拨矿电机角度环PID
#define YAW_MOTOR_ANGLE_PID_KP 3.0f 
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 11000.0f

//拨矿电机速度环PID         pitch轴     操作延迟
#define PITCH_MOTOR_SPEED_PID_KP 1000.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.0f
#define PITCH_MOTOR_SPEED_PID_KD 2000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 10000.0f

//拨矿电机角度环PID
#define PITCH_MOTOR_ANGLE_PID_KP 3.0f 
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 11000.0f



//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CATCH_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define CATCH_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

// 各电机角度限幅
#define SPIN_LIMIT_ANGLE 100.0f
#define YAW_LIMIT_ANGLE 100.0f
#define SUCTION_LIMIT_ANGLE 100.0f

#define MOTOR_SPEED_TO_CATCH_SPEED 0.25f

//拨矿过程最大速度
#define NORMAL_MAX_CATCH_SPEED 6.0f //2.0
//伸爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 2.0f //2.0

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

// 用于电机ID
typedef enum
{
    CAN_SPIN_L_MOTOR = 0,
    CAN_SPIN_R_MOTOR,
    CAN_CATCH_YAW_MOTOR,
    CAN_CATCH_SUCTION_MOTOR,
};                    

// 用于自动模式下的电机控制
typedef enum
{
    SPIN_MOTOR = 0,
    YAW_MOTOR,
    SUCTION_MOTOR,
    MOTOR_NUM,
};  

typedef enum
{
    CATCH_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    CATCH_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

    CATCH_CLOSE,                       //全自动，操作手没有权限控制

    CATCH_HOLD,

    CATCH_FLIP,

    CATCH_ROLL,

    CATCH_YAW,

} catch_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    CATCH_AUTO,

    CATCH_HAND,      //用了自动模式的都说好

    CATCH_HOLDM,

} catch_mode_e;      //控制模式



class Catch 
{
public:
    const RC_ctrl_t *catch_RC; //抓取机构使用的遥控器指针
    RC_ctrl_t *last_catch_RC; //抓取机构使用的遥控器指针

    uint16_t catch_last_key_v;  //遥控器上次按键

    catch_behaviour_e catch_behaviour_mode; //抓取机构行为状态机
    catch_behaviour_e last_catch_behaviour_mode; //抓取机构上次行为状态机

    catch_mode_e catch_mode; //抓取机构控制状态机
    catch_mode_e last_catch_mode; //抓取机构上次控制状态机

    Mine_motor catch_motive_motor[4];
    int32_t moto_start_angle[4];

    uint8_t motor_status[MOTOR_NUM];
    fp32 catch_hold_angle[4];

    
    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();

    void auto_control(auto_mode_e *auto_mode);

    void motor_angle_limit(Mine_motor *motor);
    //行为模式
    void behaviour_control_set(fp32 *vcatch_set, fp32 *vyaw_set, fp32 *vsuction_set);

    void catch_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set);

    void motor_set_control(Mine_motor *motor);

    void solve();

    void output();

};


extern Catch minecatch;


#endif