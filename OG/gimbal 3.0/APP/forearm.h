/*
    清朝老工程义肢安装
    该文件yaw实际为pitch，懒得改了
*/
#ifndef FOREARM_H
#define FOREARM_H

//头文件
#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"
#include "CAN_receive.h"
#include "gpio.h"
#include "tim.h"
#include "catch_auto.h"
#include "DMPower.h"

//------------------------达妙------------------------
#define DAMIAO_ROLL  0x01   //达妙roll轴电机
#define DAMIAO_PITCH 0x02   //达妙pitch轴电机
#define DAMIAO_YAW  0x03    //达妙yaw轴电机

#define DAMIAO_ROLL_SPEED 0.02
#define DAMIAO_PITCH_SPEED 0.02
#define DAMIAO_YAW_SPEED 0.02

#define DM_ROLL_1_ANGLE_MAX 1.7
#define DM_ROLL_1_ANGLE_MIN -1.7

#define DM_PITCH_2_ANGLE_MAX 3.0
#define DM_PITCH_2_ANGLE_MIN -3.0

#define DM_YAW_3_ANGLE_MAX 0.45
#define DM_YAW_3_ANGLE_MIN -3.0


#define DM_DELAY 5


//大喵电机结构体 同下forearm.can，基本没用

typedef struct //TODO待补充电机读取数据
{
    int8_t  state;
    float angle_target;

}damiao_can;


//小臂 大结构体 出爪
typedef struct
{
    //遥控器指针
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[6];
    const reset_t *reset_key;
    //函数指针定义

    //电机电流、电机状态存储
    struct
    {
        //出爪
        int16_t stretch_L;
        int16_t stretch_R;

        int16_t stretch_target;
        fp32    stretch_lenth;
        int16_t stretch_speed_L;
        int16_t stretch_speed_R;
        int8_t  stretch_state;
        
        //小臂横移
        int16_t slid;
        int16_t slid_target;
        int16_t slid_speed;
        int8_t  slid_state;
        
        //小臂roll 转矿
        int16_t roll;
        int16_t roll_target;
        int16_t roll_speed;
        int8_t  roll_state;

        //小臂yaw
        int16_t yaw;
        int16_t yaw_target;
        int16_t yaw_speed;
        int8_t  yaw_state;
        int8_t  last_yaw_state;
        int16_t yaw_speed_target;


    }can;
    //达妙电机
    damiao_can damiao_roll;
    damiao_can damiao_pitch;
    damiao_can damiao_yaw;
   

    float flip_angle;
    float flip_stay_angle;
    float flip_reset_angle;

    
    float yaw_angle;
    float yaw_stay_angle;
    float yaw_reset_angle;
    
    float stretch_lenth;//出爪长度
    int16_t reset_flag;
    int64_t reset_last_flag;

    //遥控器状态命�?
    #define left_switch_is_up           (forearm.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (forearm.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (forearm.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (forearm.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (forearm.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (forearm.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (forearm.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (forearm.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (forearm.rc_data->rc.ch[3] == 0)

    //出爪状态
    #define stretch_state_is_stop       (forearm.can.stretch_state == stop)
    #define stretch_state_is_out        (forearm.can.stretch_state == stretch_out)
    #define stretch_state_is_back       (forearm.can.stretch_state == stretch_back)

    //横移状态
    #define slid_state_is_stop       (forearm.can.slid_state == stop)
    #define slid_state_is_right      (forearm.can.slid_state == forward)
    #define slid_state_is_left       (forearm.can.slid_state == reverse)

    //小臂roll转矿状态
    #define roll_state_is_stop          (forearm.can.roll_state == stop)
    #define roll_state_is_forward       (forearm.can.roll_state == forward)
    #define roll_state_is_reverse       (forearm.can.roll_state == reverse)

    //小臂yaw状态
    #define yaw_state_is_stop          (forearm.can.yaw_state == stop)
    #define yaw_state_is_forward       (forearm.can.yaw_state == forward)
    #define yaw_state_is_reverse       (forearm.can.yaw_state == reverse)


    //翻爪状态
    #define flip_state_is_stop       (forearm.can.flip_state == stop)
    #define flip_state_is_up         (forearm.can.flip_state == forward)//暂定
    #define flip_state_is_down       (forearm.can.flip_state == reverse)

}forearm_ctrl_t;

//出爪位置限位
fp32 stretch_out_lenth = 1050.0f;
fp32 stretch_back_lenth = 70.0f;
fp32 flip_angle_up = 250.0f;
fp32 flip_angle_down = -5.0f;
//PID状态
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
}forearm_state;


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

} pid_strt;

//一号电机PID   小臂roll
float FOREARM_ROLL_KP     =   10.0f;
float FOREARM_ROLL_KI     =   0.0f;
float FOREARM_ROLL_KD     =   0.0f;
float FOREARM_ROLL_MOUT   =   2000.0f;
float FOREARM_ROLL_MIOUT  =   1.0f;
//二号电机PID   小臂横移
float FOREARM_SLID_KP     =   10.0f;
float FOREARM_SLID_KI     =   0.0f;
float FOREARM_SLID_KD     =   0.0f;
float FOREARM_SLID_MOUT   =   2000.0f;
float FOREARM_SLID_MIOUT  =   1.0f;
//三号电机PID   出爪
float FOREARM_STRETCH_L_KP     =   15.0f;
float FOREARM_STRETCH_L_KI     =   0.0f;
float FOREARM_STRETCH_L_KD     =   0.0f;
float FOREARM_STRETCH_L_MOUT   =   2000.0f;
float FOREARM_STRETCH_L_MIOUT  =   1.0f;
//五号电机PID   出爪
float FOREARM_STRETCH_R_KP     =   10.0f;
float FOREARM_STRETCH_R_KI     =   0.0f;
float FOREARM_STRETCH_R_KD     =   5.0f;
float FOREARM_STRETCH_R_MOUT   =   2000.0f;
float FOREARM_STRETCH_R_MIOUT  =   1.0f;

//翻爪角度环
float CATCH_ANGLE_KP     =   60.0f;
float CATCH_ANGLE_KI     =   0.0f;
float CATCH_ANGLE_KD     =   5.0f;
float CATCH_ANGLE_MOUT   =   400.0f;
float CATCH_ANGLE_MIOUT  =   1.0f;
//四号电机PID   小臂yaw
float FOREARM_YAW_KP     =   10.0f;
float FOREARM_YAW_KI     =   0.0f;
float FOREARM_YAW_KD     =   0.0f;
float FOREARM_YAW_MOUT   =   10000.0f;
float FOREARM_YAW_MIOUT  =   1.0f;

//addmotor05



void forearm_set_mode(void);
void forearm_control(void);
void forearm_can_send(void);
void DM_set_sent(void);
void forearm_flip_can_send(void);
fp32 forearm_PID_calc(pid_strt *pid, int16_t ref, int16_t set);
void forearm_PID_init(void);
void forearm_init(void);
void auto_ore(void);

#endif
