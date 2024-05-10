#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_can.h"
#include "can.h"
#include "struct_typedef.h"
#include "DMPower.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_elbow_motor_measure(uint8_t num, uint8_t data[8])
{
    elbow_motor[num].last_ecd = elbow_motor[num].ecd;
    elbow_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    elbow_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    elbow_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    elbow_motor[num].temperate = data[6];
    elbow_motor[num].last_total_angle = elbow_motor[num].total_angle;
    //计圈
    if (elbow_motor[num].ecd - elbow_motor[num].last_ecd > 4096)
        elbow_motor[num].round_cnt -- ;
    else if (elbow_motor[num].ecd - elbow_motor[num].last_ecd < -4096)
        elbow_motor[num].round_cnt ++ ;   
    //增量式角度环计算
    elbow_motor[num].total_angle = (elbow_motor[num].round_cnt * 360.0f) / 19.0f + elbow_motor[num].ecd * 360.0f / 19.0f / 8192.0f;
    elbow_motor[num].angle_err = elbow_motor[num].last_total_angle - elbow_motor[num].total_angle;
}

void Can_receive::get_terminal_motor_measure(uint8_t num, uint8_t data[8])
{
    terminal_motor[num].last_ecd = terminal_motor[num].ecd;
    terminal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    terminal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    terminal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    terminal_motor[num].temperate = data[6];
    terminal_motor[num].last_total_angle = terminal_motor[num].total_angle;
    //计圈
    if (terminal_motor[num].ecd - terminal_motor[num].last_ecd > 4096)
        terminal_motor[num].round_cnt -- ;
    else if (terminal_motor[num].ecd - terminal_motor[num].last_ecd < -4096)
        terminal_motor[num].round_cnt ++ ;   
    //增量式角度环计算
    terminal_motor[num].total_angle = (terminal_motor[num].round_cnt * 360.0f) / 19.0f  + terminal_motor[num].ecd * 360.0f / 19.0f / 8192.0f;
    terminal_motor[num].angle_err = terminal_motor[num].last_total_angle - terminal_motor[num].total_angle;
}



/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x205) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x206) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x207) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x208) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_arm_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    arm_tx_message.StdId = CAN_CATCH_MOTIVE_ALL_ID;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x08;
    arm_can_send_data[0] = motor1 >> 8;
    arm_can_send_data[1] = motor1;
    arm_can_send_data[2] = motor2 >> 8;
    arm_can_send_data[3] = motor2;
    arm_can_send_data[4] = motor3 >> 8;
    arm_can_send_data[5] = motor3;
    arm_can_send_data[6] = motor4 >> 8;
    arm_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CATCH_CAN, &arm_tx_message, arm_can_send_data, &send_mail_box);
}

/**
 * @brief          返回拨矿电机和伸出电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_elbow_motor_measure_point(uint8_t i)
{
    return &elbow_motor[i];
}

/**
 * @brief          返回抓取结构各电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_terminal_motor_measure_point(uint8_t i)
{
    return &terminal_motor[i];
}

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
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

void Can_receive::send_high_state_com(bool_t stretch_state , bool_t yaw_state, bool_t roll_state, bool_t flip_state){
        
    top_send.stretch_state = stretch_state;
    top_send.yaw_state = yaw_state;
    top_send.roll_state = roll_state;
    top_send.flip_state = flip_state;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_UI_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = stretch_state;
    can_send_data[1] = yaw_state;
    can_send_data[2] = roll_state;
    can_send_data[3] = flip_state;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);

}

void Can_receive::send_ss_state_com(uint8_t s0, uint8_t s1,int16_t ch1,bool_t sucker_flag){

    top_send.s0 = s0;
    top_send.s1 = s1;
    top_send.ch_1 = ch1;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SS_BOARD_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = s0;
    can_send_data[1] = s1;
    can_send_data[2] = ch1 >> 8;
    can_send_data[3] = ch1;
    can_send_data[4] = sucker_flag;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_lift_auto_mode(int8_t auto_mode){

        top_send.auto_mode = auto_mode;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SEND_LIFT_AUTOMODE_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = auto_mode>> 8;
    can_send_data[1] = auto_mode;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);

}

void Can_receive::receive_lift_auto_state(uint8_t data[8]){

    top_receive.lift_state = (int16_t)(data[0] << 8 | data[1]);
}
