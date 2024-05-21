#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"
#include "CAN_receive.h"

typedef struct 
{
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[5];

    struct 
    {
        int16_t motor1;//从左前轮开始顺时针电机1-4
        int16_t motor2;
        int16_t motor3;
        int16_t motor4;
        int16_t motor1_target;
        int16_t motor2_target;
        int16_t motor3_target;
        int16_t motor4_target;
        int16_t motor1_speed;
        int16_t motor2_speed;
        int16_t motor3_speed;
        int16_t motor4_speed;
        int16_t chassis_state;
        float   translation_ratio;
        float   rotating_ratio;
        int16_t vx;
        int16_t vy;
        int16_t vz;
    }can;

    #define     RIGHT_SWITCH_UP         (chassis.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (chassis.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (chassis.rc_data->rc.s[0] == 2)
    #define     LEFT_SWITCH_UP          (chassis.rc_data->rc.s[1] == 1)
    #define     LEFT_SWITCH_MID         (chassis.rc_data->rc.s[1] == 3)
    #define     LEFT_SWITCH_DOWN        (chassis.rc_data->rc.s[1] == 2)
    
    #define     SHUT_DOWN               (chassis.can.chassis_state == shut)
    #define     STANDARD                (chassis.can.chassis_state == standard)
    #define     FAST                    (chassis.can.chassis_state == fast)

}chassis_ctrl_t;


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

} chassis_pid_strt;


int16_t transverse;           
int16_t lengthways;            
int16_t roating;               

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum
{
    shut    =   0,
    standard,
    fast,
    crazy
}chassis_state_type;
//一号电机PID
float MOTOR1_KP     =   100.0f;
float MOTOR1_KI     =   0.0f;
float MOTOR1_KD     =   0.0f;
float MOTOR1_MOUT   =   16000.0f;
float MOTOR1_MIOUT  =   1.0f;
//二号电机PID
float MOTOR2_KP     =   100.0f;
float MOTOR2_KI     =   0.0f;
float MOTOR2_KD     =   0.0f;
float MOTOR2_MOUT   =   16000.0f;
float MOTOR2_MIOUT  =   1.0f;
//三号电机PID
float MOTOR3_KP     =   100.0f;
float MOTOR3_KI     =   0.0f;
float MOTOR3_KD     =   0.0f;
float MOTOR3_MOUT   =   16000.0f;
float MOTOR3_MIOUT  =   1.0f;
//四号电机PID
float MOTOR4_KP     =   100.0f;
float MOTOR4_KI     =   0.0f;
float MOTOR4_KD     =   0.0f;
float MOTOR4_MOUT   =   16000.0f;
float MOTOR4_MIOUT  =   1.0f;    

int key_board = 0;
int speed_mode = 0;
int chassis_flag = 0;
int chassis_last_flag = 0;

void chassis_set_mode(void);
void chassis_control(void);
void chassis_can_send(void);
fp32 chassis_PID_calc(chassis_pid_strt *pid, int16_t ref, int16_t set);
void chassis_PID_init(void);
void chassis_init(void);

#endif
