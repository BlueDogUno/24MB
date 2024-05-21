#ifndef SAVE_H
#define SAVE_H
#include "remote_control.h"

typedef struct 
{
    const RC_ctrl_t *rc_data;

    #define     RIGHT_SWITCH_UP         (save.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (save.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (save.rc_data->rc.s[0] == 2)
    #define     LEFT_SWITCH_UP          (save.rc_data->rc.s[1] == 1)
    #define     LEFT_SWITCH_MID         (save.rc_data->rc.s[1] == 3)
    #define     LEFT_SWITCH_DOWN        (save.rc_data->rc.s[1] == 2)

    #define     LEFT_CH_UP              (save.rc_data->rc.ch[3] > 100)
    #define     LEFT_CH_DOWN            (save.rc_data->rc.ch[3] < -100)
    #define     LEFT_CH_MID             (save.rc_data->rc.ch[3] > -100 && card.rc_data->rc.ch[1] < 100)
    
    #define     GIMBAL_CTRL             (LEFT_SWITCH_DOWN&&RIGHT_SWITCH_UP)

    int16_t gimbal_angle;

}save_control_t;



save_control_t save;

#endif
