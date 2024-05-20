#include "lift.h"
lift_pid_strt lift_PID[4];
int16_t *lift_Reset_key;
strt_t    strt;
// 键盘模式值
int8_t lift_keyboard = 0;

void Lifting_task(void const * argument)
{   
    lift_init();
    while(1)
    {
        // strt.lift_lenth = 1.0*(strt.motor_measure[0]->round*360)/19+1.0*(strt.motor_measure[0]->ecd*360)/19/8192;
        
        lift_set_mode();
        lift_control();
        ore_set_mode();
        ore_control();
        vTaskDelay(1);
        can_send();
        pump_contorl_send();
        // ccan_send();
        vTaskDelay(3);
    }
}

/************函数开始*************/
// 这里使用了define判断的方式让条件更加易懂，其中switch为遥控器的左右三档开关，rocker为左右的拨杆
void lift_set_mode(void)
{
    if(left_switch_is_down&&right_switch_is_up)
    {
        lift_keyboard = 1;
    }else{
        lift_keyboard = 0;
    }

    if(lift_keyboard == 0)
    {
        if(left_switch_is_mid && right_switch_is_mid)     
        {
            if (left_rocker_up)
            {
                strt.can.lift.state = up;
            }
            else if(left_rocker_down)
            {
                strt.can.lift.state = down;
            }
            else
            {
                strt.can.lift.state = stop;
            }
        }
    }else{
        if(strt.rc_data->key.v == KEY_PRESSED_OFFSET_Z)     
        {
            if (strt.rc_data->mouse.y < 0)
            {
                strt.can.lift.state = up;
            }
            if(strt.rc_data->mouse.y > 0)
            {
                strt.can.lift.state = down;
            }
            if(strt.rc_data->mouse.y == 0)
            {
                strt.can.lift.state = stop;
            }
        }else{
            strt.can.lift.state = stop;
        }
    }
    
    if(lift_keyboard == 0)
    {
        if(left_switch_is_up && right_switch_is_up) //下上？    
        {
            if (dial_forward)
            {
                strt.pump_state = open;
            }
            else if(dial_reserve)
            {
                strt.pump_state = close;
            }
            else
            {
                strt.pump_state = close;
            }
        }
    }else{
        if(strt.rc_data->key.v == KEY_PRESSED_OFFSET_Z)    //控制按键待确定 
        {
            if (strt.rc_data->mouse.y < 0)
            {
                strt.pump_state = up;
            }
            if(strt.rc_data->mouse.y > 0)
            {
                strt.pump_state = down;
            }
            if(strt.rc_data->mouse.y == 0)
            {
                strt.pump_state = stop;
            }
        }else{
            strt.pump_state = stop;
        }
    }
    // if(strt.reset_key->Reset_key == 0)
    // {
    //     if(strt.lift_lenth > lift_down && state_is_down) //电控限位
    //     {
    //         strt.can.lift.state = stop;
    //     }   
    //     if(strt.lift_lenth < lift_up && state_is_up)
    //     {
    //         strt.can.lift.state = stop;
    //     }
    // }

}

void ore_set_mode(void)
{
    if(lift_keyboard == 0)
    {
        if(left_switch_is_mid && right_switch_is_up)     
        {
        if (left_rocker_up)//strt.sensor_data.photogate_1
        {
            strt.can.ore.state   =   in;
        }else if(left_rocker_down)//strt.sensor_data.photogate_1 == 0
        {
            strt.can.ore.state   =   out;
        }else{
            strt.can.ore.state   =   stop;
        }
        }
        else
        {
            strt.can.ore.state   =  stop;
        }
    }else{
        if(strt.rc_data->key.v == KEY_PRESSED_OFFSET_Q)
        {
            if(strt.rc_data->mouse.z > 0)
            {
                strt.can.ore.state = out;
            }else if(strt.rc_data->mouse.z < 0)
            {
                strt.can.ore.state = in;
            }else{
                strt.can.ore.state = stop;
            }
        }
    //     ore_last_flag = ore_flag;
    //     if(strt.rc_data->key.v == KEY_PRESSED_OFFSET_Q)
    //     {
    //         ore_flag = 1;
    //     }else{
    //         ore_flag = 0;
    //     }
    //     if(ore_last_flag != ore_flag && ore_flag == 1)
    //     {
    //         if(ore_state_is_in)
    //         {
    //             strt.can.ore.state = out;
    //         }else{
    //             strt.can.ore.state = in;
    //         }
    //     }
    }

}

void lift_control(void)
{   
    strt.can.lift.left_speed    = strt.motor_measure[2]->speed_rpm;
    strt.can.lift.right_speed   = strt.motor_measure[3]->speed_rpm;
    strt.can.ore.left_speed    = strt.motor_measure[2]->speed_rpm;
    strt.can.ore.right_speed   = strt.motor_measure[3]->speed_rpm;

    strt.can.lift.left  = 0;
    strt.can.lift.right = 0;

    if (state_is_stop)
    {
        strt.can.lift.left_target   =   -13 * 19;
        strt.can.lift.right_target  =   13 * 19;
    }

    if (state_is_up)
    {
        strt.can.lift.left_target   =   -60 * 19;
        strt.can.lift.right_target  =   60 * 19;
    }

    if (state_is_down)
    {
        strt.can.lift.left_target   =   10*19;//40 * 19;
        strt.can.lift.right_target  =   -10*19;//-40 * 19;
    }

    if (state_is_shut)
    {
        strt.can.lift.left_target   =   -13 * 19;
        strt.can.lift.right_target  =   13 * 19;
    }

    // if(strt.auto_behave->target_mode == 1 && lift_keyboard == 1/*&& strt.auto_behave->a_takein_mode == 0*/ ) //自动模式
    // {
    //     if(strt.lift_lenth - strt.auto_behave->a_lift_target > 5.0f)
    //     {
    //         strt.can.lift.left_target   =   -60 * 19;
    //         strt.can.lift.right_target  =   60 * 19;
    //     }
    //     if(strt.lift_lenth - strt.auto_behave->a_lift_target < -5.0f)
    //     {
    //         strt.can.lift.left_target   =   10*19;
    //         strt.can.lift.right_target  =   -10*19;
    //     }
    //     if(strt.lift_lenth - strt.auto_behave->a_lift_target < 5.0f && strt.lift_lenth - strt.auto_behave->a_lift_target > -5.0f)
    //     {
    //         strt.can.lift.left_target   =   -13 * 19;
    //         strt.can.lift.right_target  =   13 * 19;
    //     }
    // }
    
    strt.can.lift.left = (int16_t)lift_PID_calc(&lift_PID[0],strt.can.lift.left_speed,strt.can.lift.left_target);
    strt.can.lift.right = (int16_t)lift_PID_calc(&lift_PID[1],strt.can.lift.right_speed,strt.can.lift.right_target);
    
}

void ore_control(void)
{
    if (ore_state_is_stop)
    {
        strt.can.ore.left_target   =   0;
        strt.can.ore.right_target  =   0;
    }

    if (ore_state_is_in)
    {
        strt.can.ore.left_target   =   10000;
        strt.can.ore.right_target  =   -10000;
    }

    if (ore_state_is_out)
    {
        strt.can.ore.left_target   =   -10000;
        strt.can.ore.right_target  =   10000;
    }
    strt.can.ore.left = (int16_t)lift_PID_calc(&lift_PID[2],strt.can.ore.left_speed,strt.can.ore.left_target);
    strt.can.ore.right = (int16_t)lift_PID_calc(&lift_PID[3],strt.can.ore.right_speed,strt.can.ore.right_target);
}

// void sensor(void)
// {
//     if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
//     {
//         strt.sensor_data.photogate_1  =   0;
//     }

//     if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
//     {
//         strt.sensor_data.photogate_1  =   1;
//     }
    
// }
// 自动模式使用位置环和速度环来控制
// void lift_auto_control(void)
// {
//     strt.can.lift.left_target = (int16_t)lift_PID_calc(&lift_PID[4],(int16_t)strt.lift_lenth,(int16_t)strt.auto_behave->a_lift_target);
//     strt.can.lift.right_target = -1*strt.can.lift.left_target;
    
//     strt.can.lift.left = (int16_t)lift_PID_calc(&lift_PID[0],strt.can.lift.left_speed,strt.can.lift.left_target);
//     strt.can.lift.right = (int16_t)lift_PID_calc(&lift_PID[1],strt.can.lift.right_speed,strt.can.lift.right_target);
// }

// can发送电流值
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              lift_can_send_data[8];
void can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    lift_can_send_data[0] = 0 >> 8;
    lift_can_send_data[1] = 0;
    lift_can_send_data[2] = 0 >> 8;
    lift_can_send_data[3] = 0;
    // lift_can_send_data[4] = 0 >> 8;
    // lift_can_send_data[5] = 0;
    // lift_can_send_data[6] = 0 >> 8;
    // lift_can_send_data[7] = 0;
    lift_can_send_data[4] = strt.can.lift.left >> 8;
    lift_can_send_data[5] = strt.can.lift.left;
    lift_can_send_data[6] = strt.can.lift.right >> 8;
    lift_can_send_data[7] = strt.can.lift.right;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, lift_can_send_data, &send_mail_box);
}

// static CAN_TxHeaderTypeDef  ccan_tx_message;
// static uint8_t              tlift_can_send_data[8];
// void ccan_send(void)
// {
    // uint32_t send_mail_box;
    // ccan_tx_message.StdId = 0x1FF;
    // ccan_tx_message.IDE = CAN_ID_STD;
    // ccan_tx_message.RTR = CAN_RTR_DATA;
    // ccan_tx_message.DLC = 0x08;
    // tlift_can_send_data[0] = strt.can.lift.left >> 8;
    // tlift_can_send_data[1] = strt.can.lift.left;
    // tlift_can_send_data[2] = strt.can.lift.right >> 8;
    // tlift_can_send_data[3] = strt.can.lift.right;
    // tlift_can_send_data[4] = 0;
    // tlift_can_send_data[5] = 0;
    // tlift_can_send_data[6] = 0;
    // tlift_can_send_data[7] = 0;

    // HAL_CAN_AddTxMessage(&hcan2, &ccan_tx_message, tlift_can_send_data, &send_mail_box);
// }

// PID数组的赋值，即pid初始化
void lift_PID_init(void)
{
    lift_PID[0].mode = PID_POSITION;
    lift_PID[0].Kp = LIFT_LEFT_KP;
    lift_PID[0].Ki = LIFT_LEFT_KI;
    lift_PID[0].Kd = LIFT_LEFT_KD;
    lift_PID[0].max_out = LIFT_LEFT_MOUT;
    lift_PID[0].max_iout = LIFT_LEFT_MIOUT;
    lift_PID[0].Dbuf[0] = lift_PID[0].Dbuf[1] = lift_PID[0].Dbuf[2] = 0.0f;
    lift_PID[0].error[0] = lift_PID[0].error[1] = lift_PID[0].error[2] = lift_PID[0].Pout = lift_PID[0].Iout = lift_PID[0].Dout = lift_PID[0].out = 0.0f;

    lift_PID[1].mode = PID_POSITION;
    lift_PID[1].Kp = LIFT_RIGHT_KP;
    lift_PID[1].Ki = LIFT_RIGHT_KI;
    lift_PID[1].Kd = LIFT_RIGHT_KD;
    lift_PID[1].max_out = LIFT_RIGHT_MOUT;
    lift_PID[1].max_iout = LIFT_RIGHT_MIOUT;
    lift_PID[1].Dbuf[0] = lift_PID[1].Dbuf[1] = lift_PID[1].Dbuf[2] = 0.0f;
    lift_PID[1].error[0] = lift_PID[1].error[1] = lift_PID[1].error[2] = lift_PID[1].Pout = lift_PID[1].Iout = lift_PID[1].Dout = lift_PID[1].out = 0.0f;

    lift_PID[2].mode = PID_POSITION;
    lift_PID[2].Kp = ORE_LEFT_KP;
    lift_PID[2].Ki = ORE_LEFT_KI;
    lift_PID[2].Kd = ORE_LEFT_KD;
    lift_PID[2].max_out = ORE_LEFT_MOUT;
    lift_PID[2].max_iout = ORE_LEFT_MIOUT;
    lift_PID[2].Dbuf[0] = lift_PID[2].Dbuf[1] = lift_PID[2].Dbuf[2] = 0.0f;
    lift_PID[2].error[0] = lift_PID[2].error[1] = lift_PID[2].error[2] = lift_PID[2].Pout = lift_PID[2].Iout = lift_PID[2].Dout = lift_PID[2].out = 0.0f;

    lift_PID[3].mode = PID_POSITION;
    lift_PID[3].Kp = ORE_RIGHT_KP;
    lift_PID[3].Ki = ORE_RIGHT_KI;
    lift_PID[3].Kd = ORE_RIGHT_KD;
    lift_PID[3].max_out = ORE_RIGHT_MOUT;
    lift_PID[3].max_iout = ORE_RIGHT_MIOUT;
    lift_PID[3].Dbuf[0] = lift_PID[3].Dbuf[1] = lift_PID[3].Dbuf[2] = 0.0f;
    lift_PID[3].error[0] = lift_PID[3].error[1] = lift_PID[3].error[2] = lift_PID[3].Pout = lift_PID[3].Iout = lift_PID[3].Dout = lift_PID[3].out = 0.0f;
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

fp32 lift_PID_calc(lift_pid_strt *pid, int16_t ref, int16_t set)
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
//万恶的PID至此结束


void lift_init(void)
{
    //遥控器初始化
    strt.rc_data    =   get_remote_control_point();
    
    //状态初始化
    strt.pump_state = close;
    strt.can.lift.state = stop;
    strt.can.ore.state = stop;

    //电机初始化
    for (uint8_t i = 0; i < 6; i++)
    {
        strt.motor_measure[i] = get_motor_measure_point(i+4);
    }

    // strt.auto_behave = get_auto_control_point();
    strt.reset_key = get_reset_point();
    lift_PID_init();
    
    //抬升高度初始化
    strt.lift_lenth = 0.0f;
}

//气泵控制
void pump_contorl_send(void){
        if (strt.pump_state == open)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
        }
        else if (strt.pump_state == close)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); 
        } 

}