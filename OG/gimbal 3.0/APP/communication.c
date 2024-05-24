#include "communication.h"
#include "remote_control.h"
#include "CAN_receive.h"

#include "main.h"
#include "string.h"

Top_send_t top_send_t;
RC_ctrl_t rc_ctrl_t;

void communication_task(void const * argument)
{
    run();
    
}

void run()
{

    rc_ctrl_t.rc.ch[0] = top_send_t.ch_0;
    rc_ctrl_t.rc.ch[1] = top_send_t.ch_1;
    rc_ctrl_t.rc.ch[2] = top_send_t.ch_2;
    rc_ctrl_t.rc.ch[3] = top_send_t.ch_3;
    rc_ctrl_t.key.v = top_send_t.v;
    rc_ctrl_t.rc.s[1] = top_send_t.s1;
    rc_ctrl_t.rc.s[0] = top_send_t.s0;
}
