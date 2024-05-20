#ifndef CAN_receive_H
#define CAN_receive_H

#include "struct_typedef.h"

#define BOARD_COM_CAN hcan2




typedef enum
{
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

} can_msg_id_can2_e;

typedef enum
{
    CAN_LIFT_LEFT_ID = 0x201,//[4]
    CAN_LIFT_RIGHT_ID = 0x202,//[5]
    CAN_ORE_LEFT_ID = 0x203,//[6]
    CAN_ORE_RIGHT_ID = 0x204,//[7]
    
    CAN_DAMIAO_ID = 0x01,//һ�Ŵ�����
    CAN_DAMIAO_ROLL_ID = 0x301, //һ���İ汾�������id
    CAN_RC_BOARM_COM_ID = 0x302,//板件通讯ID
} can_msg_id_can1_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct 
{
    int16_t Reset_key;
    int16_t Reset_last_key;
    int16_t Reset_flag;
    int16_t Reset_last_flag;
}reset_t;

typedef struct {
  //遥控器数据
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint16_t v;
    uint8_t s0;
    uint8_t s1;

    bool_t stretch_state;
    bool_t yaw_state;
    bool_t roll_state;
    bool_t flip_state; 

    int8_t auto_mode;

}Top_send_t;


extern const motor_measure_t *get_motor_measure_point(uint8_t i);
extern const reset_t *get_reset_point(void);
extern void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v);
#endif
