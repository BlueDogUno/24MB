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


#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    // TODO 设备检测未更新
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        // if (hcan == &CHASSIS_CAN) //接底盘CAN 信息
        // {

        //     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        //     switch (rx_header.StdId)
        //     {
        //     //底盘动力电机
        //     case CAN_MOTIVE_FR_MOTOR_ID:
        //         can_receive.get_motive_motor_measure(MOTIVE_FR_MOTOR, rx_data);
        //         // detect_hook(CHASSIS_MOTIVE_FR_MOTOR_TOE);
        //         break;
        //     case CAN_MOTIVE_FL_MOTOR_ID:
        //         can_receive.get_motive_motor_measure(MOTIVE_FL_MOTOR, rx_data);
        //         // detect_hook(CHASSIS_MOTIVE_FL_MOTOR_TOE);
        //         break;
        //     case CAN_MOTIVE_BL_MOTOR_ID:
        //         can_receive.get_motive_motor_measure(MOTIVE_BL_MOTOR, rx_data);
        //         // detect_hook(CHASSIS_MOTIVE_BL_MOTOR_TOE);
        //         break;
        //     case CAN_MOTIVE_BR_MOTOR_ID:
        //         can_receive.get_motive_motor_measure(MOTIVE_BR_MOTOR, rx_data);
        //         // detect_hook(CHASSIS_MOTIVE_BR_MOTOR_TOE);
        //         break;

        //     //底盘抬升电机
        //     case CAN_LIFT_LEFT_MOTOR_ID:
        //         can_receive.get_lift_motor_measure(LIFT_LEFT_MOTOR, rx_data);
        //         //detect_hook(CHASSIS_RUDDER_FR_MOTOR_TOE);
        //         break;
            
        //     case CAN_LIFT_RIGHT_MOTOR_ID:
        //         can_receive.get_lift_motor_measure(LIFT_RIGHT_MOTOR, rx_data);
        //         //detect_hook(CHASSIS_RUDDER_FL_MOTOR_TOE);
        //         break;
        //     default:
        //     {
        //         break;
        //     }
        //     }
        // }


        if (hcan == &BOARD_COM_CAN) //接底盘CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {

            case CAN_RC_BOARM_COM_ID:
                receive_rc_board_com(rx_data);
                // detect_hook(BOARD_COM);
                break;

            case CAN_SS_BOARD_COM_ID:
                can_receive.receive_ss_board_com(rx_data);
                // detect_hook(BOARD_COM);
                break;
            // case CAN_UI_COM_ID:
            //     can_receive.receive_ui_board_com(rx_data);
            //     break;
            default:
            {
                break;
            }
            }
        }
}
#endif