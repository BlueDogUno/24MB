#include "communication.h"
#include "CAN_receive.h"
#include "remote_control.h"

const RC_ctrl_t *rc_data;

void communication_task(void const * argument)
{
    run();
}


void run()
{
    rc_data = get_remote_control_point();
    int16_t temp_ch0, temp_ch2, temp_ch3,temp_ch1;
    uint16_t temp_v;
    int8_t s0,s1;


    temp_ch0 = rc_data->rc.ch[0];
    temp_ch1 = rc_data->rc.ch[1];
    temp_ch2 = rc_data->rc.ch[2];
    temp_ch3 = rc_data->rc.ch[3];
    temp_v = rc_data->key.v;
    s0 = rc_data->rc.s[0];
    s1 = rc_data->rc.s[1];

    send_rc_board_com(temp_ch0, temp_ch2, temp_ch3, temp_v);

}
