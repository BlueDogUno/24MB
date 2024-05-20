#include "communication.h"
#include "remote_control.h"
#include "CAN_receive.h"

Top_send_t top_send_t;
RC_ctrl_t rc_ctrl_t;
void run()
{
    // referee.unpack();
    // referee.determine_ID();

    // ui.run();

    // //向云台发送裁判数据
    // bool_t lift_state;

    // lift_state = high.motor_status[0];

    // can_receive.send_lift_auto_state(lift_state);
    
// TODO _data这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    rc_ctrl_t.rc.ch[0] = top_send_t.ch_0;
    rc_ctrl_t.rc.ch[1] = top_send_t.ch_1;
    rc_ctrl_t.rc.ch[2] = top_send_t.ch_2;
    rc_ctrl_t.rc.ch[3] = top_send_t.ch_3;
    rc_ctrl_t.key.v = top_send_t.v;
    rc_ctrl_t.rc.s[1] = top_send_t.s1;
    rc_ctrl_t.rc.s[0] = top_send_t.s0;
    // remote_control.rc_ctrl.mouse.z = can_receive.chassis_receive.z;
#else
    ;
#endif
}