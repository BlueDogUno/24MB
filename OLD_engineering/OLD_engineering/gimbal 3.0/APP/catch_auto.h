#ifndef catch_AUTO_H
#define catch_AUTO_H
#include "struct_typedef.h"
#include "remote_control.h" 
#include "CAN_receive.h"

typedef struct 
{
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor[9];

    int16_t auto_mode;
    int16_t target_mode;
    int16_t press_flag;
    int16_t last_press_flag;
    int16_t photogate_1;
    float flip_reset_angle;
    int16_t flip_reset_flag;
    int16_t flip_reset_last_flag;
    //0为停�?1为上�?2为下
    int16_t a_catch_mode;
    int16_t a_takein_mode;
    int16_t a_takeout_mode;
    int16_t a_push_mode;
    int16_t a_exchange_mode;

    int16_t a_push_flag;
    int16_t a_takein_flag;

    float a_lift_target;
    float a_stretch_target;
    int16_t a_flip_target;
    int16_t a_catch_target;

    int16_t arrive;

    float   a_flip_angle;
    float   a_lift_angle;
    float   a_stretch_angle;


    float a_lift_down;
    float a_lift_up;
    float a_stretch_out;
    float a_stretch_back;
    float a_flip_up;
    float a_flip_down;

}auto_t;

#define STOP 0
#define UP 1
#define DOWN 2
#define MID 3

#define CLOSE 1
#define OPEN 2

#define OUT 1
#define BACK 2

#define lift_STOP       (auto_ctrl.a_lift_mode == STOP)
#define lift_UP         (auto_ctrl.a_lift_mode == UP)
#define lift_DOWN       (auto_ctrl.a_lift_mode == DOWN)


extern const auto_t *get_auto_control_point(void);

#endif
