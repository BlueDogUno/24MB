#ifndef __DMPOWER__
#define __DMPOWER__

#include "DMPower.h"
#include "stm32f4xx_hal.h"
#define MOTOR1  0x01   //1号电机ID
#define MOTOR2  0x02   //2号电机ID
#define MOTOR3  0x03   //3号电机ID
#define MOTOR4  0x04   //4号电机ID

#define P_MIN   -12.5  //λ����Сֵ
#define P_MAX   12.5   //λ�����ֵ
#define V_MIN   -45    //�ٶ���Сֵ
#define V_MAX   45     //�ٶ����ֵ
#define KP_MIN  0      //Kp��Сֵ
#define KP_MAX  500    //Kp���ֵ
#define KD_MIN  0      //Kd��Сֵ
#define KD_MAX  5      //Kd���ֵ
#define T_MIN   -18    //ת�����ֵ
#define T_MAX   18     //ת����Сֵ

//����ṹ��
typedef struct
{
  int p_int;
  int v_int;
  int t_int;
  int position;
  int velocity;
  int torque;
}Motor_t;
//���Ͱ�
typedef struct
{
	CAN_TxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_TxPacketTypeDef;
//���հ�
typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;

void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
void Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

#endif /* __DMPOWER__ */
