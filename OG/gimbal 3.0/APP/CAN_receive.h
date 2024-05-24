#ifndef CAN_receive_H
#define CAN_receive_H

#include "struct_typedef.h"


#define BOARD_COM_CAN hcan1
#define CHASSIS_CAN hcan2

typedef struct 
{
    int16_t Reset_key;
    int16_t Reset_last_key;
    int16_t Reset_flag;
    int16_t Reset_last_flag;
}reset_t;


typedef enum
{
    CAN_ROLL_ID = 0x201,//[0]
    CAN_SLID_ID = 0x202,//[1]
    CAN_STRETCH_L_ID = 0x203,//[2]
    CAN_YAW_ID = 0x204,//[3] 
    CAN_STRETCH_R_ID = 0x205,//[4]

} can_msg_id_can2_e;

//addmotor03

typedef enum
{
    CAN_LIFT_LEFT_ID = 0x201,//[4]
    CAN_LIFT_RIGHT_ID = 0x202,//[5]
    CAN_ORE_LEFT_ID = 0x203,//[6]
    CAN_ORE_RIGHT_ID = 0x204,//[7]
    
    CAN_DAMIAO_ID = 0x01,//һ�Ŵ�����
    CAN_DAMIAO_ROLL_ID = 0x301, //һ���İ汾�������id

} can_msg_id_can1_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t round;
} motor_measure_t;

typedef struct
{
    uint8_t dis1;
    uint16_t dis2;
    uint8_t dis_status;
    uint16_t signal_strength;
    int16_t dis0;
    float dis;


} sensor_measure_t;


//�������ṹ��------------
typedef struct
{
    //�����������
    int p_int;
    int v_int;
    int t_int;
    int position;
    int velocity;
    int torque;

    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t round;
    uint8_t pcb_temperate;

}damiao_maesure_t;

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

typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,

  //实际id 由于不可抗力硬件问题没办法改 把遥控器正负改了一下  勉强用用  
  // CAN_MOTIVE_FL_MOTOR_ID = 0x201,
  // CAN_MOTIVE_FR_MOTOR_ID = 0x202,  
  // CAN_MOTIVE_BR_MOTOR_ID = 0x203,
  // CAN_MOTIVE_BL_MOTOR_ID = 0x204,

  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //抬升电机ID CAN2
  CAN_LIFT_LEFT_MOTOR_ID = 0x205,
  CAN_LIFT_RIGHT_MOTOR_ID = 0x206,

  
  CAN_CHASSIS_LIFT_ALL_ID = 0x1FF,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x301,
  CAN_SS_BOARD_COM_ID = 0x302,
  CAN_SEND_LIFT_AUTO_COM_ID = 0x303,
  // CAN_UI_COM_ID = 0x305,

} can_msg_id_e;
//���Ͱ�
// typedef struct
// {
// 	CAN_TxHeaderTypeDef hdr;
// 	uint8_t payload[8];
// }CAN_TxPacketTypeDef;
// //���հ�
// typedef struct
// {
// 	CAN_RxHeaderTypeDef hdr;
// 	uint8_t payload[8];
// }CAN_RxPacketTypeDef;
// =========================================


extern const motor_measure_t *get_motor_measure_point(uint8_t i);
extern const reset_t *get_reset_point(void);
extern const sensor_measure_t *get_sensor_measure_point(uint8_t i);
extern void can_receive_init(void);
void receive_rc_board_com(uint8_t data[8]);
void receive_ss_board_com(uint8_t data[8]);
#endif
