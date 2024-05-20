#include "card.h"

int a=0;

void card_task(void const * argument)
{
  card_init();
  while(1)
  {
    card_set_mode();
    card_control();
    card_can_send();
  }
}


card_pid_strt card_pid;
card_ctrl_t card;
void card_set_mode(void)
{
    if(LEFT_SWITCH_MID && RIGHT_SWITCH_DOWN)
    {
        if(LEFT_CH_MID)
        {
            card.can.state  =   stop;
        }
        if(LEFT_CH_UP)
        {
            card.can.state  =   forward;
        }
        if(LEFT_CH_DOWN)
        {
            card.can.state  =   reverse;
        }
    }else if(card.rc_data->key.v == KEY_PRESSED_OFFSET_V){
        if(card.rc_data->mouse.z > 0)
        {
            card.can.state  =   forward;
        }
        if(card.rc_data->mouse.z < 0)
        {
            card.can.state  =   reverse;
        }
    }else{
        card.can.state  =   stop;
    }
}

void card_control(void)
{
    card.can.card_speed = card.motor_measure[4]->speed_rpm;

    if(card_state_stop)
    {
        card.can.card_target   =   0;
    }
    if(card_state_forward)
    {
        card.can.card_target   =   -20 * 36;
    }
    if(card_state_reverse)
    {
        card.can.card_target   =   10 * 36;
    }
    card.can.card = (int16_t)card_PID_calc(&card_pid,card.can.card_speed,card.can.card_target) - 1000;

    if(card_state_shut)
    {
        card.can.card = 0;
    }
}

static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              card_can_send_data[8];

void card_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x1FF;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    card_can_send_data[0] = card.can.card >> 8;
    card_can_send_data[1] = card.can.card;
    card_can_send_data[2] = 0 >> 8;
    card_can_send_data[3] = 0;
    card_can_send_data[4] = 0 >> 8;
    card_can_send_data[5] = 0;
    card_can_send_data[6] = 0 >> 8;
    card_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, card_can_send_data, &send_mail_box);
}

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

fp32 card_PID_calc(card_pid_strt *pid, int16_t ref, int16_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void card_PID_init(void)
{
    card_pid.mode = PID_POSITION;
    card_pid.Kp = CARD_KP;
    card_pid.Ki = CARD_KI;
    card_pid.Kd = CARD_KD;
    card_pid.max_out = CARD_MOUT;
    card_pid.max_iout = CARD_MIOUT;
    card_pid.Dbuf[0] = card_pid.Dbuf[1] = card_pid.Dbuf[2] = 0.0f;
    card_pid.error[0] = card_pid.error[1] = card_pid.error[2] = card_pid.Pout = card_pid.Iout = card_pid.Dout = card_pid.out = 0.0f;
}

void card_init(void)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        card.motor_measure[i] = get_motor_measure_point(i);
    }

    card_PID_init();

}
