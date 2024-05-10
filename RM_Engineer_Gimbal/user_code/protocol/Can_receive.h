#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"
#include "DMPower.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


#define ARM_COM hcan2
#define DM_ARM_COM hcan1


/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MINE_FR_MOTOR_ID = 0x201,
  CAN_MINE_FL_MOTOR_ID = 0x202,
  CAN_STRETCH_FR_MOTOR_ID = 0x203,
  CAN_STRETCH_FL_MOTOR_ID = 0x204,

  CAN_MINE_MOTIVE_ALL_ID = 0x200,

  CAN_SPIN_R_MOTOR_ID = 0x205,
  CAN_SPIN_L_MOTOR_ID = 0x206,
  CAN_CATCH_YAW_MOTOR_ID = 0x207,
  CAN_CATCH_SUCTION_MOTOR_ID = 0x208,

  CAN_CATCH_MOTIVE_ALL_ID = 0x1FF,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x301,
  CAN_SS_BOARD_COM_ID = 0x302,
  CAN_SEND_LIFT_AUTOSTATE_COM_ID = 0x303,
  CAN_SEND_LIFT_AUTOMODE_COM_ID = 0x304,
  CAN_UI_COM_ID = 0x305,

} can_msg_id_e;


typedef enum
{
    CAN_ROLL_ID = 0x201,//[0]
    CAN_Y_SLID_ID = 0x202,//[1]
    CAN_X_STRETCH_ID = 0x203,//[2]

} can_msg_id_can2_e;


typedef enum
{
  CAN_DAMIAO_YAW = 0x02,
  CAN_DAMIAO_PITCH = 0x04,

} can_msg_id_can1_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;

  uint16_t offset_angle;     //补偿角度
	fp32  round_cnt;        //转子转动圈数
	fp32  total_angle;      //转子转动总角度
	fp32  last_total_angle;
	fp32  angle_err;
} motor_measure_t;

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

typedef struct {

    bool_t lift_state;

}Top_receive_t;

class Can_receive
{
public:
  //动力电机反馈数据结构体
  motor_measure_t elbow_motor[4];
  motor_measure_t terminal_motor[4];

  Top_send_t  top_send;
  Top_receive_t top_receive;

  //发送数据结构体
  CAN_TxHeaderTypeDef arm_tx_message;
  CAN_TxHeaderTypeDef can_tx_message;

  uint8_t arm_can_send_data[8];
  uint8_t can_send_data[8];

  void init();

  //电机数据接收
  void get_elbow_motor_measure(uint8_t num, uint8_t data[8]);

  void get_terminal_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_arm_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //臂姿态位置电机can发送

  const motor_measure_t *get_terminal_motor_measure_point(uint8_t i);

  const motor_measure_t *get_elbow_motor_measure_point(uint8_t i);
 
 //发送遥控器数据
  void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v);           

  //发送顶端状态
  void send_high_state_com(bool_t stretch_state , bool_t yaw_state, bool_t roll_state, bool_t flip_state);

  //发送拨杆数据
  void send_ss_state_com(uint8_t s0, uint8_t s1,int16_t ch1,bool_t sucker_flag);  // void send_ss_state_com(uint8_t s0, uint8_t s1);

  //发送底盘自动抬升行为状态
  void send_lift_auto_mode(int8_t auto_mode);

  //接收底盘自动抬升状态
  void receive_lift_auto_state(uint8_t data[8]);
};

#endif
