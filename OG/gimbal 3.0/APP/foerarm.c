#include "forearm.h"
#include "CAN_receive.h"
#include "DMPower.h"
#include "remote_control.h"

void forearm_task(void const *argument)
{
    forearm_init();             //初始化函数指针及参数
    while(1)
    {
        forearm_set_mode();     //更改遥控器控制模式
        forearm_control();      //更改电机控制模式
        forearm_can_send();     //CAN协议发送

        PhotoSpin_set_sent();
        PhotoSpin_output();

        software_reset();

        // DM_set_sent();
        // forearm_flip_can_send();  //翻爪can协议
    }
}
forearm_ctrl_t forearm;
PhotoSpin photospin;
pid_strt forearm_PID[7];
int16_t servo_data;

// int8_t  auto_get;//取矿
// int8_t  auto_in;//收矿
// int8_t  auto_out;//出矿
// int8_t  auto_ec;//换矿


// void auto_ore(void)
// {
//     auto_get = 0;
//     auto_in = 0;
//     auto_out = 0;
//     auto_ec = 0;
// }


// 0为遥控器模式，1为键盘模式
int8_t forearm_keyboard = 1;
int8_t forearm_forearm_flag = 0;
int8_t forearm_forearm_last_flag = 0;

//更改遥控器控制模式


void forearm_set_mode(void)
{
    // if(left_switch_is_mid&&right_switch_is_down)
    // {
    //     forearm_keyboard = 1;
    // }else{
        forearm_keyboard = 0;
    // }

    //出爪 X键
    if(forearm_keyboard == 0)
    {
        // 遥控器模式
        if (left_switch_is_down && right_switch_is_up)
        {
            //起始赋值
            if(left_rocker_up)
            {
                forearm.can.stretch_state = stretch_out;
            }
            if(left_rocker_down)
            {
                forearm.can.stretch_state = stretch_back;
            }
            if(left_rocker_mid)
            {
                forearm.can.stretch_state = stop;
            }
        }
        else
        {
            forearm.can.stretch_state =   stop;
        }
    }else{
        // 键盘模式
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        {
            //起始赋值
            if(forearm.rc_data->mouse.y < 0)
            {
                forearm.can.stretch_state = stretch_out;
            }
            if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.stretch_state = stretch_back;
            }
            if(forearm.rc_data->mouse.y == 0)
            {
                forearm.can.stretch_state = stop;
            }
        }
        else
        {
            forearm.can.stretch_state =  stop;
        }
    }
    // 电控限位
    // if(forearm.reset_key->Reset_flag == 0)
    // {
    //     if(forearm.stretch_lenth < stretch_back_lenth && stretch_state_is_back)
    //     {
    //         forearm.can.stretch_state =   stop;
    //     }
    //     if(forearm.stretch_lenth > stretch_out_lenth && stretch_state_is_out)
    //     {
    //         forearm.can.stretch_state =   stop;
    //     }
    // }
    
    //roll轴达妙旋转
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_up)
        {
            if (left_rocker_up)
            {
                forearm.damiao_roll.state = forward; 
            }
            if (left_rocker_down)
            {
                forearm.damiao_roll.state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.damiao_roll.state =  stop;
            }
        }
        else {
            forearm.damiao_roll.state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.damiao_roll.state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.damiao_roll.state =  reverse;

            }else
            {
                forearm.damiao_roll.state =  stop;
            }
        }
        else
        {
            forearm.damiao_roll.state =   stop;
        }   
    }

    //pitch轴达妙旋转
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_mid)
        {
            if (left_rocker_up)
            {
                forearm.damiao_pitch.state = forward;
            }
            if (left_rocker_down)
            {
                forearm.damiao_pitch.state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.damiao_pitch.state =  stop;
            }
        }
        else {
            forearm.damiao_pitch.state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.damiao_pitch.state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.damiao_pitch.state =  reverse;

            }else
            {
                forearm.damiao_pitch.state =  stop;
            }
        }
        else
        {
            forearm.damiao_pitch.state =   stop;
        }   
    }
    //yaw轴达妙旋转
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                forearm.damiao_yaw.state = forward; 
            }
            if (left_rocker_down)
            {
                forearm.damiao_yaw.state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.damiao_yaw.state =  stop;
            }
        }
        else {
            forearm.damiao_yaw.state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.damiao_yaw.state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.damiao_yaw.state =  reverse;

            }else
            {
                forearm.damiao_yaw.state =  stop;
            }
        }
        else
        {
            forearm.damiao_yaw.state =   stop;
        }   
    }

    //小臂2006横移
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_down && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                forearm.can.slid_state = forward;
            }
            if (left_rocker_down)
            {
                forearm.can.slid_state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.can.slid_state =  stop;
            }
        }
        else {
            forearm.can.slid_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_Q)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.slid_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.slid_state =  reverse;

            }else
            {
                forearm.can.slid_state =  stop;
            }
        }
        else
        {
            forearm.can.slid_state =   stop;
        }   
    }

    //小臂2006转矿
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_mid && right_switch_is_up)
        {
            if (left_rocker_up)
            {
                forearm.can.roll_state = forward;
            }
            if (left_rocker_down)
            {
                forearm.can.roll_state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.can.roll_state =  stop;
            }
        }
        else {
            forearm.can.roll_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_G)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.roll_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.roll_state =  reverse;

            }else
            {
                forearm.can.roll_state =  stop;
            }
        }
        else
        {
            forearm.can.roll_state =   stop;
        }   
        

    }
    
    DM_set_sent();

    // //小臂3508翻爪
    // if(forearm_keyboard == 0)
    // {
    //     if (left_switch_is_mid && right_switch_is_down)
    //     {
    //         if (left_rocker_up)
    //         {
    //             forearm.can.flip_state = forward;
    //         }
    //         if (left_rocker_down)
    //         {
    //             forearm.can.flip_state =  reverse;
    //         }
    //         if(left_rocker_mid)
    //         {
    //             forearm.can.flip_state =  stop;
    //         }
    //     }
    //     else {
    //         forearm.can.flip_state =  stop;
    //     }
    // }else{
    //     if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_B)
    //     {
    //         if (forearm.rc_data->mouse.y < 0)
    //         {
    //            forearm.can.flip_state =  forward;

    //         }else if(forearm.rc_data->mouse.y > 0)
    //         {
    //             forearm.can.flip_state =  reverse;

    //         }else
    //         {
    //             forearm.can.flip_state =  stop;
    //         }
    //     }
    //     else
    //     {
    //         forearm.can.flip_state =   stop;
    //     }   
    // }


    //小臂2006yaw轴
    forearm.can.last_yaw_state = forearm.can.yaw_state;
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_mid && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                forearm.can.yaw_state = reverse;
            }
            if (left_rocker_down)
            {
                forearm.can.yaw_state = forward;
            }
            if(left_rocker_mid)
            {
                forearm.can.yaw_state =  stop;
            }
        }
        else {
            forearm.can.yaw_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.yaw_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.yaw_state =  reverse;

            }else
            {
                forearm.can.yaw_state =  stop;
            }
        }
        else
        {
            forearm.can.yaw_state =   stop;
        }   
    }
    // if(catch.reset_key->Reset_flag == 0)     //限位，待上机读数据
    // {
    //     if(catch.flip_angle >= flip_angle_up && flip_state_is_forward)
    //     {
    //         catch.can.flip_state =   stop;
    //     }
    //     if(catch.flip_angle <= flip_angle_down && flip_state_is_reverse)
    //     {
    //         catch.can.flip_state =   stop;
    //     }
        if(forearm.can.last_yaw_state != forearm.can.yaw_state)
        {
            forearm.yaw_stay_angle = forearm.yaw_angle;
        }
    // }
    //cnm图传视角移动
    if(forearm_keyboard == 0)
    {
        // 遥控器模式
        if (left_switch_is_mid && right_switch_is_down)
        {
            //起始赋值
            if(left_rocker_up)
            {
                photospin.photospin_state = LOOK_UP;

            }else if(left_rocker_down)
            {
                photospin.photospin_state = LOOK_DOWN;

            }else if(left_rocker_right)
            {
                photospin.photospin_state = LOOK_RIGHT;

            }else if(left_rocker_left)
            {
                photospin.photospin_state = LOOK_LEFT;
            }else 
            {
                photospin.photospin_state =   LOOK_STOP;
            }
        }
        else
        {
            photospin.photospin_state =   LOOK_STOP;
        }
    }else{
        photospin.photospin_state =   LOOK_STOP;
        // 键盘模式
        // if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        // {
        //     //起始赋值
        //     if(forearm.rc_data->mouse.y < 0)
        //     {
        //         photospin.photospin_state = LOOK_UP;
        //     }
        //     if(forearm.rc_data->mouse.y > 0)
        //     {
        //         photospin.photospin_state = LOOK_DOWN;
        //     }
        //     if(forearm.rc_data->mouse.y == 0)
        //     {
        //         photospin.photospin_state = stop;
        //     }
        //     if(forearm.rc_data->mouse.x < 0)
        //     {
        //         photospin.photospin_state = LOOK_LEFT;
        //     }
        //     if(forearm.rc_data->mouse.x > 0)
        //     {
        //         photospin.photospin_state = LOOK_RIGHT;
        //     }
        //     if(forearm.rc_data->mouse.x == 0)
        //     {
        //         photospin.photospin_state = stop;
        //     }
        // }
        // else
        // {
        //     photospin.photospin_state =  stop;
        // }
    }

    //addmotor01

}

//更改电机控制模式
void forearm_control(void)
{ 
    //接收电机状态
    forearm.can.roll_speed = forearm.motor_measure[0]->speed_rpm;
    forearm.can.slid_speed = forearm.motor_measure[1]->speed_rpm;
    forearm.can.stretch_speed_L = forearm.motor_measure[2]->speed_rpm;
    forearm.can.stretch_speed_R = forearm.motor_measure[4]->speed_rpm;
    forearm.can.yaw_speed = forearm.motor_measure[3]->speed_rpm;
    // forearm.can.flip_right_speed = forearm.motor_measure[4]->speed_rpm;
    //addmotor02


// 出爪
    if (stretch_state_is_stop)
    {
        forearm.can.stretch_target   =   0;
    }

    if (stretch_state_is_out)
    {   
        forearm.can.stretch_target   =   160 * 19;
    }
    if (stretch_state_is_back)
    {
        forearm.can.stretch_target   =   -160 * 19;
    }
    //控制出爪电机
    forearm.can.stretch_L = (int16_t)forearm_PID_calc(&forearm_PID[2],forearm.can.stretch_speed_L,forearm.can.stretch_target);

    forearm.can.stretch_R = (int16_t)forearm_PID_calc(&forearm_PID[6],forearm.can.stretch_speed_R,-forearm.can.stretch_target);
    


    //小臂横移
    if (slid_state_is_stop)
    {
        forearm.can.slid_target   =   0;
    }

    if (slid_state_is_right)
    {   
        forearm.can.slid_target   =   160 * 19;
    }
    if (slid_state_is_left)
    {
        forearm.can.slid_target   =   -160 * 19;
    }
    forearm.can.slid = (int16_t)forearm_PID_calc(&forearm_PID[1],forearm.can.slid_speed,forearm.can.slid_target);
    

    //小臂roll
    if (roll_state_is_stop)
    {
        forearm.can.roll_target   =   0 ;//TODO保持

    }

    if (roll_state_is_forward)
    {   
        forearm.can.roll_target   =   60 * 15;
    }
    if (roll_state_is_reverse)
    {
        forearm.can.roll_target   =   -60 * 15;
    }
    forearm.can.roll = (int16_t)forearm_PID_calc(&forearm_PID[0],forearm.can.roll_speed,forearm.can.roll_target);
    
    //小臂yaw
    //todo holdon_test
    if (yaw_state_is_stop)
    {
        forearm.can.yaw_speed_target  =(int16_t)forearm_PID_calc(&forearm_PID[5],(int16_t)forearm.yaw_angle,(int16_t)forearm.yaw_stay_angle);
        // forearm.can.yaw_target   =   0 ;

    }
    if (yaw_state_is_forward)
    {   
        forearm.can.yaw_speed_target   =   20 * 19;
    }
    if (yaw_state_is_reverse)
    {
        forearm.can.yaw_speed_target   =   -20 * 19;
    }
    forearm.can.yaw = (int16_t)forearm_PID_calc(&forearm_PID[3],forearm.can.yaw_speed,forearm.can.yaw_speed_target);
    forearm.yaw_angle = 1.0*(forearm.motor_measure[3]->round*360)/19+1.0*(forearm.motor_measure[3]->ecd*360)/19/8192;
    
    // if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    // {
    //     forearm.can.yaw = 0;
    // }

    //addmotor06


}

void DM_set_sent(void){
    //达妙roll  DAMIAO_ROLL 
    if (forearm.damiao_roll.state ==   stop)
    {

    }else if (forearm.damiao_roll.state ==  forward)
    {   
        forearm.damiao_roll.angle_target += DAMIAO_ROLL_SPEED;

    }else if (forearm.damiao_roll.state ==  reverse)
    {
        forearm.damiao_roll.angle_target -= DAMIAO_ROLL_SPEED;
    }
    if (forearm.damiao_roll.angle_target > DM_ROLL_1_ANGLE_MAX )
    {
        forearm.damiao_roll.angle_target = DM_ROLL_1_ANGLE_MAX;

    }else if (forearm.damiao_roll.angle_target <DM_ROLL_1_ANGLE_MIN)
    {
        forearm.damiao_roll.angle_target = DM_ROLL_1_ANGLE_MIN;
    }
    MIT_CtrlMotor(&hcan1,DAMIAO_ROLL,forearm.damiao_roll.angle_target,0,2,1,0);
    HAL_Delay(DM_DELAY);

    
    //达妙pitch DAMIAO_PITCH    
    if (forearm.damiao_pitch.state ==   stop)
    {

    }else if (forearm.damiao_pitch.state ==  forward)
    {   
        forearm.damiao_pitch.angle_target += DAMIAO_PITCH_SPEED;

    }else if (forearm.damiao_pitch.state ==  reverse)
    {
        forearm.damiao_pitch.angle_target -= DAMIAO_PITCH_SPEED;
    }
    if (forearm.damiao_pitch.angle_target > DM_PITCH_2_ANGLE_MAX )
    {
        forearm.damiao_pitch.angle_target = DM_PITCH_2_ANGLE_MAX;

    } else if (forearm.damiao_pitch.angle_target <DM_PITCH_2_ANGLE_MIN)
    {
        forearm.damiao_pitch.angle_target = DM_PITCH_2_ANGLE_MIN;
    }
    MIT_CtrlMotor(&hcan1,DAMIAO_PITCH,forearm.damiao_pitch.angle_target,0,2,1,0);
    HAL_Delay(DM_DELAY);

    //达妙yaw DAMIAO_YAW  
    if (forearm.damiao_yaw.state ==   stop)
    {

    }else if (forearm.damiao_yaw.state ==  forward)
    {   
        forearm.damiao_yaw.angle_target += DAMIAO_YAW_SPEED;

    }else if (forearm.damiao_yaw.state ==  reverse)
    {
        forearm.damiao_yaw.angle_target -= DAMIAO_YAW_SPEED;
    }
    if (forearm.damiao_yaw.angle_target > DM_YAW_3_ANGLE_MAX )
    {
        forearm.damiao_yaw.angle_target = DM_YAW_3_ANGLE_MAX;

    } else if (forearm.damiao_yaw.angle_target <DM_YAW_3_ANGLE_MIN)
    {
        forearm.damiao_yaw.angle_target = DM_YAW_3_ANGLE_MIN;
    }
    MIT_CtrlMotor(&hcan1,DAMIAO_YAW,forearm.damiao_yaw.angle_target,0,2,1,0);
    HAL_Delay(DM_DELAY);
}
// void catch_auto_control(void)


//CAN协议发送
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              forearm_can_send_data[8];
void forearm_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    forearm_can_send_data[0] = forearm.can.roll >> 8;
    forearm_can_send_data[1] = forearm.can.roll;
    forearm_can_send_data[2] = forearm.can.slid >> 8;
    forearm_can_send_data[3] = forearm.can.slid;
    forearm_can_send_data[4] = forearm.can.stretch_L >> 8;
    forearm_can_send_data[5] = forearm.can.stretch_L;
    forearm_can_send_data[6] = forearm.can.yaw >> 8;
    forearm_can_send_data[7] = forearm.can.yaw;

    //HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, forearm_can_send_data, &send_mail_box);
	  if(HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, forearm_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, forearm_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, forearm_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}


//出抓电机
static CAN_TxHeaderTypeDef  can_tx_stretch_message;
static uint8_t              forearm_stretch_can_send_data[8];
void forearm_stretch_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_stretch_message.StdId = 0x1FF;
    can_tx_stretch_message.IDE = CAN_ID_STD;
    can_tx_stretch_message.RTR = CAN_RTR_DATA;
    can_tx_stretch_message.DLC = 0x08;
    forearm_stretch_can_send_data[0] = forearm.can.stretch_R >> 8;
    forearm_stretch_can_send_data[1] = forearm.can.stretch_R;
    forearm_stretch_can_send_data[2] = 0;
    forearm_stretch_can_send_data[3] = 0;
    forearm_stretch_can_send_data[4] = 0;
    forearm_stretch_can_send_data[5] = 0;
    forearm_stretch_can_send_data[6] = 0;
    forearm_stretch_can_send_data[7] = 0;

   // HAL_CAN_AddTxMessage(&hcan2, &can_tx_stretch_message, forearm_stretch_can_send_data, &send_mail_box);
	  if(HAL_CAN_AddTxMessage(&hcan2, &can_tx_stretch_message, forearm_stretch_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(&hcan2, &can_tx_stretch_message, forearm_stretch_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(&hcan2, &can_tx_stretch_message, forearm_stretch_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}
//addmotor07

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

fp32 forearm_PID_calc(pid_strt *pid, int16_t ref, int16_t set)
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


void forearm_PID_init(void)
{

    forearm_PID[0].mode = PID_POSITION;
    forearm_PID[0].Kp = FOREARM_ROLL_KP;
    forearm_PID[0].Ki = FOREARM_ROLL_KI;
    forearm_PID[0].Kd = FOREARM_ROLL_KD;
    forearm_PID[0].max_out = FOREARM_ROLL_MOUT;
    forearm_PID[0].max_iout = FOREARM_ROLL_MIOUT;
    forearm_PID[0].Dbuf[0] = forearm_PID[0].Dbuf[1] = forearm_PID[0].Dbuf[2] = 0.0f;
    forearm_PID[0].error[0] = forearm_PID[0].error[1] = forearm_PID[0].error[2] = forearm_PID[0].Pout = forearm_PID[0].Iout = forearm_PID[0].Dout = forearm_PID[0].out = 0.0f;


    forearm_PID[1].mode = PID_POSITION;
    forearm_PID[1].Kp = FOREARM_SLID_KP;
    forearm_PID[1].Ki = FOREARM_SLID_KI;
    forearm_PID[1].Kd = FOREARM_SLID_KD;
    forearm_PID[1].max_out = FOREARM_SLID_MOUT;
    forearm_PID[1].max_iout = FOREARM_SLID_MIOUT;
    forearm_PID[1].Dbuf[0] = forearm_PID[1].Dbuf[1] = forearm_PID[1].Dbuf[2] = 0.0f;
    forearm_PID[1].error[0] = forearm_PID[1].error[1] = forearm_PID[1].error[2] = forearm_PID[1].Pout = forearm_PID[1].Iout = forearm_PID[1].Dout = forearm_PID[1].out = 0.0f;


    forearm_PID[2].mode = PID_POSITION;
    forearm_PID[2].Kp = FOREARM_STRETCH_L_KP;
    forearm_PID[2].Ki = FOREARM_STRETCH_L_KI;
    forearm_PID[2].Kd = FOREARM_STRETCH_L_KD;
    forearm_PID[2].max_out = FOREARM_STRETCH_L_MOUT;
    forearm_PID[2].max_iout = FOREARM_STRETCH_L_MIOUT;
    forearm_PID[2].Dbuf[0] = forearm_PID[2].Dbuf[1] = forearm_PID[2].Dbuf[2] = 0.0f;
    forearm_PID[2].error[0] = forearm_PID[2].error[1] = forearm_PID[2].error[2] = forearm_PID[2].Pout = forearm_PID[2].Iout = forearm_PID[2].Dout = forearm_PID[2].out = 0.0f;

    forearm_PID[5].mode = PID_POSITION;
    forearm_PID[5].Kp = CATCH_ANGLE_KP;
    forearm_PID[5].Ki = CATCH_ANGLE_KI;
    forearm_PID[5].Kd = CATCH_ANGLE_KD;
    forearm_PID[5].max_out = CATCH_ANGLE_MOUT;
    forearm_PID[5].max_iout = CATCH_ANGLE_MIOUT;
    forearm_PID[5].Dbuf[0] = forearm_PID[5].Dbuf[1] = forearm_PID[5].Dbuf[2] = 0.0f;
    forearm_PID[5].error[0] = forearm_PID[5].error[1] = forearm_PID[5].error[2] = forearm_PID[5].Pout = forearm_PID[5].Iout = forearm_PID[5].Dout = forearm_PID[5].out = 0.0f;

    forearm_PID[3].mode = PID_POSITION;
    forearm_PID[3].Kp = FOREARM_YAW_KP;
    forearm_PID[3].Ki = FOREARM_YAW_KI;
    forearm_PID[3].Kd = FOREARM_YAW_KD;
    forearm_PID[3].max_out = FOREARM_YAW_MOUT;
    forearm_PID[3].max_iout = FOREARM_YAW_MIOUT;
    forearm_PID[3].Dbuf[0] = forearm_PID[3].Dbuf[1] = forearm_PID[3].Dbuf[2] = 0.0f;
    forearm_PID[3].error[0] = forearm_PID[3].error[1] = forearm_PID[3].error[2] = forearm_PID[3].Pout = forearm_PID[3].Iout = forearm_PID[3].Dout = forearm_PID[3].out = 0.0f;

    forearm_PID[6].mode = PID_POSITION;
    forearm_PID[6].Kp = FOREARM_STRETCH_R_KP;
    forearm_PID[6].Ki = FOREARM_STRETCH_R_KI;
    forearm_PID[6].Kd = FOREARM_STRETCH_R_KD;
    forearm_PID[6].max_out = FOREARM_STRETCH_R_MOUT;
    forearm_PID[6].max_iout = FOREARM_STRETCH_R_MIOUT;
    forearm_PID[6].Dbuf[0] = forearm_PID[0].Dbuf[1] = forearm_PID[0].Dbuf[2] = 0.0f;
    forearm_PID[6].error[0] = forearm_PID[0].error[1] = forearm_PID[0].error[2] = forearm_PID[0].Pout = forearm_PID[0].Iout = forearm_PID[0].Dout = forearm_PID[0].out = 0.0f;


    //addmotor08
}


//初始化函数指针及参数
void forearm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    forearm.rc_data   =   get_remote_control_point();
    forearm.reset_key = get_reset_point();
    
    forearm_PID_init();

    for (uint8_t i = 0; i < 6; i++)
    {
        forearm.motor_measure[i] = get_motor_measure_point(i);
    }
    forearm.can.stretch_state =   stop;
    forearm.can.slid_state = stop;
    forearm.can.roll_state = stop;
    forearm.can.yaw_state = stop;
    forearm.damiao_roll.state =  stop;
    forearm.damiao_pitch.state = stop;
    forearm.damiao_yaw.state =  stop;
    forearm.stretch_lenth = 0.0f;
    
    forearm.damiao_roll.angle_target = 0.0f;
    forearm.damiao_pitch.angle_target = 0.0f;
    forearm.damiao_yaw.angle_target = 0.0f;

    forearm.yaw_stay_angle = forearm.yaw_angle;
    forearm.yaw_angle = 0;

    photospin.photospin_state = LOOK_MID;
    photospin.pitch_angle = PS_PITCH_MID_ANGLE;
    photospin.yaw_angle = PS_YAW_MID_ANGLE;
    //addmotor09

    servo_data = 1500;
    //达妙电机使能
    //注意can终端电阻配置，和can发送时间间隔。可能会影响电机的使能

    Enable_CtrlMotor(&hcan1,DAMIAO_ROLL,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    HAL_Delay(800);
    Enable_CtrlMotor(&hcan1,DAMIAO_PITCH,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    HAL_Delay(800);
    Enable_CtrlMotor(&hcan1,DAMIAO_YAW,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    HAL_Delay(800);
}


const forearm_ctrl_t *get_forearm_control_point(void)
{
    return &forearm;
}
 
 //软件重启按键延时
uint16_t software_reset_key_delay_time = 0;
/**
  * @brief          通过按键,进行重启
  * @param[in]    
  * @retval         返回空
  * @waring         
  */
void software_reset()
{
    //软件复位,让单片机重启  同时按下Z X CTRL 一秒
    if(KEY_PRESSED_OFFSET_Z && KEY_PRESSED_OFFSET_X && KEY_PRESSED_OFFSET_CTRL)
    {   
       software_reset_key_delay_time++;
    }

    if(software_reset_key_delay_time >= 2000)
    { 
        NVIC_SystemReset();
        software_reset_key_delay_time = 0;
    }
}

void PhotoSpin_set_sent(void){
    //图传 PITCH轴
    if (photospin.photospin_state ==   LOOK_MID)
    {

    }else if (photospin.photospin_state ==  LOOK_UP)
    {   
        photospin.pitch_angle += PHOTOSPIN_PITCH_SPEED;

    }else if (photospin.photospin_state ==  LOOK_DOWN)
    {
        photospin.pitch_angle -= PHOTOSPIN_PITCH_SPEED;
    }else if (photospin.photospin_state ==  LOOK_RIGHT)
    {   
        photospin.yaw_angle += PHOTOSPIN_YAW_SPEED;

    }else if (photospin.photospin_state ==  LOOK_LEFT)
    {
        photospin.yaw_angle -= PHOTOSPIN_YAW_SPEED;
    }
    

    // if (photospin.pitch_angle > PS_PITCH_ANGLE_MAX )
    // {
    //     photospin.pitch_angle = PS_PITCH_ANGLE_MAX;

    // }if (photospin.pitch_angle < PS_PITCH_ANGLE_MIN)
    // {
    //     photospin.pitch_angle = PS_PITCH_ANGLE_MIN;
    // }

    // if (photospin.yaw_angle > PS_YAW_ANGLE_MAX )
    // {
    //     photospin.yaw_angle = PS_YAW_ANGLE_MAX;

    // }if (photospin.yaw_angle < PS_YAW_ANGLE_MIN)
    // {
    //     photospin.yaw_angle = PS_YAW_ANGLE_MIN;
    // }

    

}

void PhotoSpin_output(void){


    // if(photospin.photospin_state == LOOK_UP || photospin.photospin_state == LOOK_DOWN){

	// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, photospin.pitch_angle);

    // }else if(photospin.photospin_state == LOOK_RIGHT || photospin.photospin_state == LOOK_LEFT){

	// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,photospin.yaw_angle);

    // }
    if (photospin.photospin_state == LOOK_MID){

		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PS_PITCH_MID_ANGLE);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PS_YAW_MID_ANGLE);
    }else {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, photospin.pitch_angle);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,photospin.yaw_angle);
    }
    
}