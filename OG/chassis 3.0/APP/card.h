#ifndef CARD_H
#define CARD_H
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
        int16_t card;
        int16_t card_speed;
        int16_t card_target;
        uint8_t state;
    }can;
    

    #define     RIGHT_SWITCH_UP         (card.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (card.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (card.rc_data->rc.s[0] == 2)
    #define     LEFT_SWITCH_UP          (card.rc_data->rc.s[1] == 1)
    #define     LEFT_SWITCH_MID         (card.rc_data->rc.s[1] == 3)
    #define     LEFT_SWITCH_DOWN        (card.rc_data->rc.s[1] == 2)

    #define     LEFT_CH_UP              (card.rc_data->rc.ch[3] > 100)
    #define     LEFT_CH_DOWN            (card.rc_data->rc.ch[3] < -100)
    #define     LEFT_CH_MID             (card.rc_data->rc.ch[3] > -100 && card.rc_data->rc.ch[1] < 100)
    #define     card_state_stop         (card.can.state  ==  stop)
    #define     card_state_forward      (card.can.state  ==  forward)
    #define     card_state_reverse      (card.can.state  ==  reverse)
    #define     card_state_shut         (card.can.state  ==  shut)

}card_ctrl_t;

enum state_enum
{
    stop = 0,
    forward,
    reverse,
    shut
};

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

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

} card_pid_strt;


//一号电机PID
float CARD_KP     =   11.0f;
float CARD_KI     =   0.0f;
float CARD_KD     =   0.1f;
float CARD_MOUT   =   4000.0f;
float CARD_MIOUT  =   1.0f;


void card_set_mode(void);
void card_control(void);
void card_can_send(void);
fp32 card_PID_calc(card_pid_strt *pid, int16_t ref, int16_t set);
void card_PID_init(void);
void card_init(void);

#endif
