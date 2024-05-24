#include "CAN_receive.h"
#include "can.h"
#include "communication.h"

motor_measure_t motor[9];
reset_t Reset_s;
Top_send_t  top_send;
static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t can_send_data[8];
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }



const motor_measure_t *get_motor_measure_point(uint8_t i)
{
    return &motor[i];
}
const reset_t *get_reset_point(void)
{
    return &Reset_s;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    // static CAN_RxPacketTypeDef packet; //?????????????????

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan == &hcan1)
    {
        // switch (rx_header.StdId)
        // {
        //     case CAN_SAVE_ID:
        //     {
        //         get_motor_measure(&motor[4], rx_data);
        //         break;
        //     }

        //     default:
        //     {
        //         break;
        //     }
        // }
        // if (rx_header.StdId = CAN_SAVE_ID)
        // {
        //     // get_motor_measure(&motor[4], rx_data);
        // }else if(rx_header.StdId = CAN_LIFT_LEFT_ID){
        //     static uint8_t i = 0;
        //     //get motor id
        //     i = rx_header.StdId - CAN_3508_M1_ID+4;
        //     get_motor_measure(&motor[i], rx_data);
        // }else if (rx_header.StdId = CAN_LIFT_RIGHT_ID){
        //     static uint8_t i = 0;
        //     //get motor id
        //     i = rx_header.StdId - CAN_3508_M1_ID+4;
        //     get_motor_measure(&motor[i], rx_data);
        // }
        switch (rx_header.StdId)
        {
            case CAN_LIFT_LEFT_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_LIFT_LEFT_ID + 4;
                get_motor_measure(&motor[i], rx_data);
                // if(motor[i].ecd - motor[i].last_ecd > 5000)
                // {
                //     motor[i].round--;
                // }
                // if(motor[i].ecd - motor[i].last_ecd < -5000)
                // {
                //     motor[i].round++;
                // }
                break;
            }
            case CAN_LIFT_RIGHT_ID: //?????????????????
            case CAN_ORE_LEFT_ID:
            case CAN_ORE_RIGHT_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_LIFT_LEFT_ID + 4;
                get_motor_measure(&motor[i], rx_data);
                break;
            }
        }
    }
    if(hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor[i], rx_data);
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //Êý¾ÝÌî³ä
    top_send.ch_0 = ch_0;
    top_send.ch_2 = ch_2;
    top_send.ch_3 = ch_3;
    top_send.v = v;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_3 >> 8;
    can_send_data[5] = ch_3;
    can_send_data[6] = v >> 8;
    can_send_data[7] = v;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}