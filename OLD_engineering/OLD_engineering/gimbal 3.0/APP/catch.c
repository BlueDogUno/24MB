#include "catch.h"


void catch_task(void const *argument)
{
    catch_init();             //初始化函数指针及参数
    while(1)
    {
        electromagnet_control();
        oreflip_servo();
        catch_sensor();       //空接传感器读取
        catch_set_mode();     //更改遥控器控制模式
        catch_control();      //更改电机控制模式
        
        catch_can_send();     //CAN协议发送
    }
}
catch_ctrl_t catch;
pid_strt catch_PID[6];
int8_t oreflip_flag;
int8_t oreflip_last_flag;
int16_t servo_data;
int8_t  auto_get;//取矿
int8_t  auto_in;//收矿
int8_t  auto_out;//出矿
int8_t  auto_ec;//换矿
void auto_ore(void)
{
    auto_get = 0;
    auto_in = 0;
    auto_out = 0;
    auto_ec = 0;
}
// 0为遥控器模式，1为键盘模式
int8_t catch_keyboard = 1;
int8_t catch_catch_flag = 0;
int8_t catch_catch_last_flag = 0;
//更改遥控器控制模式
void catch_set_mode(void)
{
    if(left_switch_is_down&&right_switch_is_up)
    {
        catch_keyboard = 1;
    }else{
        catch_keyboard = 0;
    }
    //角度计圈
    //翻转 C键
    catch.can.last_flip_state = catch.can.flip_state;
    if(catch_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                catch.can.flip_state =   forward;
            }

            if (left_rocker_down)
            {
                catch.can.flip_state =   reverse;
            }

            if (left_rocker_mid)
            {
                catch.can.flip_state =   stop;
            }

        }
    }else{
        if (catch.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if (catch.rc_data->mouse.y < 0)
            {
                catch.can.flip_state =   forward;
            }else if(catch.rc_data->mouse.y > 0)
            {
                catch.can.flip_state =   reverse;
            }else{
                catch.can.flip_state =   stop;
            }

        }else{
            catch.can.flip_state =   stop;
        }   
    }
    // 
    if(catch.reset_key->Reset_flag == 0)
    {
        if(catch.flip_angle >= flip_angle_up && flip_state_is_forward)
        {
            catch.can.flip_state =   stop;
        }
        if(catch.flip_angle <= flip_angle_down && flip_state_is_reverse)
        {
            catch.can.flip_state =   stop;
        }
        if(catch.can.last_flip_state != catch.can.flip_state)
        {
            catch.flip_stay_angle = catch.flip_angle;
        }
    }

    //出爪 X键
    
    if(catch_keyboard == 0)
    {
        // 遥控器模式
        if (left_switch_is_up && right_switch_is_mid)
        {
            //起始赋值
            if(left_rocker_up)
            {
                catch.can.stretch_state = stretch_out;
            }
            if(left_rocker_down)
            {
                catch.can.stretch_state = stretch_back;
            }
            if(left_rocker_mid)
            {
                catch.can.stretch_state = stop;
            }
        }
        else
        {
            catch.can.stretch_state =   stop;
        }
    }else{
        // 键盘模式
        if (catch.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        {
            //起始赋值
            if(catch.rc_data->mouse.y < 0)
            {
                catch.can.stretch_state = stretch_out;
            }
            if(catch.rc_data->mouse.y > 0)
            {
                catch.can.stretch_state = stretch_back;
            }
            if(catch.rc_data->mouse.y == 0)
            {
                catch.can.stretch_state = stop;
            }
        }
        else
        {
            catch.can.stretch_state =   stop;
        }
    }
    // 电控限位
    if(catch.reset_key->Reset_flag == 0)
    {
        if(catch.stretch_lenth < stretch_back_lenth && stretch_state_is_back)
        {
            catch.can.stretch_state =   stop;
        }
        if(catch.stretch_lenth > stretch_out_lenth && stretch_state_is_out)
        {
            catch.can.stretch_state =   stop;
        }
    }

    //夹紧
    if(catch_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_up)
        {
            if (left_rocker_up)
            {
                catch.can.catch_state =   close;
            }
            if (left_rocker_down)
            {
                catch.can.catch_state =   open;
            }
        }
    }else{
        catch_catch_last_flag = catch_catch_flag;
        if (catch.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            catch_catch_flag = 1;
        }else{
            catch_catch_flag = 0;
        }
        if(catch_catch_last_flag != catch_catch_flag && catch_catch_flag == 1)
        {
            if(catch_state_is_open)
            {
                catch.can.catch_state =   close;
            }else{
                catch.can.catch_state =   open;
            }
        }
    }
    
    /*空接
        if (catch.catch_sensor == 0)
        {
            catch.can.catch_state =   close;

        }*/

}

//更改电机控制模式
void catch_control(void)
{
    catch.can.left_speed = catch.motor_measure[0]->speed_rpm;   
    catch.can.right_speed = catch.motor_measure[1]->speed_rpm;  
    catch.can.stretch_speed = catch.motor_measure[2]->speed_rpm;
    catch.can.catch_speed = catch.motor_measure[3]->speed_rpm; 
    catch.can.stretch_lenth = catch.sensor_measure[0]->dis;

    // 翻爪
    if (flip_state_is_stop)
    {
        // catch.can.left_speed_target  =   0;
        // catch.can.right_speed_target =   0;
        catch.can.left_speed_target  =   1*(int16_t)catch_PID_calc(&catch_PID[4],(int16_t)catch.flip_angle,(int16_t)catch.flip_stay_angle);
        catch.can.right_speed_target =   -1*catch.can.left_speed_target;

    }

    if (flip_state_is_forward)
    {
        catch.can.left_speed_target  =   20 * 19;
        catch.can.right_speed_target =   -20 * 19;
    }

    if (flip_state_is_reverse)
    {
        catch.can.left_speed_target  =   -20 * 19;
        catch.can.right_speed_target =   20 * 19;
    }

    if(catch.auto_behave->target_mode == 1 && catch_keyboard == 1)  //自动模式
    {
        catch.can.left_speed_target  =   1*(int16_t)catch_PID_calc(&catch_PID[4],(int16_t)catch.flip_angle,(int16_t)catch.auto_behave->a_flip_target);
        catch.can.right_speed_target =   -1*catch.can.left_speed_target;
    
        catch.flip_stay_angle = catch.auto_behave->a_flip_target;
    }   

    catch.can.left  = (int16_t)catch_PID_calc(&catch_PID[0],catch.can.left_speed,catch.can.left_speed_target);
    catch.can.right = (int16_t)catch_PID_calc(&catch_PID[1],catch.can.right_speed,catch.can.right_speed_target);

    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        catch.can.left = 0;
        catch.can.right = 0;
    }

    // 出抓
    if (stretch_state_is_stop)
    {
        catch.can.stretch_target   =   0;
    }

    if (stretch_state_is_out)
    {   
        catch.can.stretch_target   =   160 * 19;
    }
    if (stretch_state_is_back)
    {
        catch.can.stretch_target   =   -160 * 19;
    }

    if(catch.auto_behave->target_mode == 1 && catch_keyboard == 1) //自动模式
    {
        if(catch.stretch_lenth - catch.auto_behave->a_stretch_target < -5.0f)
        {
            catch.can.stretch_target   =   160 * 19;
        }
        if(catch.stretch_lenth - catch.auto_behave->a_stretch_target > 5.0f)
        {
            catch.can.stretch_target   =   -160 * 19;
        }
    }
    catch.can.stretch = (int16_t)catch_PID_calc(&catch_PID[2],catch.can.stretch_speed,catch.can.stretch_target);

    // 夹爪
    if (catch_state_is_open)
    {
        catch.can.catch_target = -360 * 19;//-360 * 19;
        catch.can.catch = -4500;
        servo_data = 1500;
    }
    if (catch_state_is_close)
    {
        catch.can.catch = 4500;
    } 
    if(catch.auto_behave->auto_mode == 1 && catch_keyboard == 1)   //自动模式
    {
        if(catch.auto_behave->a_catch_target == 1)
        {
            if(catch.motor_measure[3]->speed_rpm < 60 && catch.motor_measure[3]->speed_rpm > -60)
            {
                catch.can.catch = 4500;
            }else{
                catch.can.catch = 10000;
            }
            catch.can.catch_state = close;
        }else{
            if(catch.motor_measure[3]->speed_rpm < 60 && catch.motor_measure[3]->speed_rpm > -60)
            {
                catch.can.catch = -4500;
            }else{
                catch.can.catch = -10000;
            }
            catch.can.catch_state = open;
        }
    }
    //catch.can.catch = (int16_t)catch.PID_calc(&catch_PID[3],catch.can.catch_speed,catch.can.catch_target);
    
    //if (catch_state_is_shut)
    //{
    //    catch.can.catch    =   0;
    //}
}


// void catch_auto_control(void)
// {
//     // 翻爪
//     catch.can.left_speed_target  =   1*(int16_t)catch.PID_calc(&catch_PID[4],(int16_t)catch.flip_angle,(int16_t)catch.auto_behave->a_flip_target);
//     catch.can.right_speed_target =   -1*catch.can.left_speed_target;
//     catch.can.left  = (int16_t)catch.PID_calc(&catch_PID[0],catch.can.left_speed,catch.can.left_speed_target);
//     catch.can.right = (int16_t)catch.PID_calc(&catch_PID[1],catch.can.right_speed,catch.can.right_speed_target);

//     // 出抓
//     catch.can.stretch = (int16_t)catch.PID_calc(&catch_PID[2],catch.can.stretch_speed,catch.can.stretch_target);

//     if(catch.auto_behave->a_catch_target == 1)
//     {
//         catch.can.catch = -4500;
//     }else{
//         catch.can.catch = 4500;
//     }

// }

//CAN协议发送
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              catch_can_send_data[8];
void catch_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    catch_can_send_data[0] = catch.can.left >> 8;
    catch_can_send_data[1] = catch.can.left;
    catch_can_send_data[2] = catch.can.right >> 8;
    catch_can_send_data[3] = catch.can.right;
    catch_can_send_data[4] = catch.can.stretch >> 8;
    catch_can_send_data[5] = catch.can.stretch;
    catch_can_send_data[6] = catch.can.catch >> 8;
    catch_can_send_data[7] = catch.can.catch;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, catch_can_send_data, &send_mail_box);
}

//夹爪空接传感器
void catch_sensor(void)
{
    // if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
    // {
    //     catch.catch_sensor  =   0;
    // }
    // if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
    // {
    //     catch.catch_sensor  =   1;
    // }

    catch.reset_last_flag = catch.reset_flag;
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        catch.reset_flag = 1;
        //catch.flip_reset_angle = catch.flip_angle;        
    }else{
        catch.reset_flag = 0;
    }
    if(catch.reset_last_flag != catch.reset_flag && catch.reset_flag == 0)
    {
        catch.flip_reset_angle = catch.flip_angle;
    }

    catch.flip_angle = 1.0*(catch.motor_measure[0]->round*360)/19+1.0*(catch.motor_measure[0]->ecd*360)/19/8192;
    catch.stretch_lenth = 1.0*(catch.motor_measure[2]->round*360)/19+1.0*(catch.motor_measure[2]->ecd*360)/19/8192;
}

void oreflip_servo(void)
{
    oreflip_last_flag = oreflip_flag;
    // if(catch.rc_data->mouse.z > 0)
    // {
    //     oreflip_flag = 1;
    // }
    // if(catch.rc_data->mouse.z == 0)
    // {
    //     oreflip_flag = 0;
    // }
    // if(catch.rc_data->mouse.z < 0)
    // {
    //     oreflip_flag = -1;
    // }
    oreflip_flag = catch.rc_data->mouse.z;
    if(catch.rc_data->key.v == KEY_PRESSED_OFFSET_E)
    {
        if(catch.rc_data->mouse.z > 0 && oreflip_last_flag != oreflip_flag)
        {
            servo_data += 200;
        }
        if(catch.rc_data->mouse.z < 0 && oreflip_last_flag != oreflip_flag)
        {
            servo_data -= 200;
        }
        if(servo_data > 2500)
        {
            servo_data = 2500;
        }
        if(servo_data < 500)
        {
            servo_data = 500;
        }
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_data);
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

fp32 catch_PID_calc(pid_strt *pid, int16_t ref, int16_t set)
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

int16_t electromagnet_v;
void electromagnet_control(void)
{
    if (left_switch_is_mid && right_switch_is_mid)
    {
        electromagnet_v = 1;
    }else{
        electromagnet_v = 0;
    }

    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, electromagnet_v);
}

void catch_PID_init(void)
{
    catch_PID[0].mode = PID_POSITION;
    catch_PID[0].Kp = CATCH_LEFT_KP;
    catch_PID[0].Ki = CATCH_LEFT_KI;
    catch_PID[0].Kd = CATCH_LEFT_KD;
    catch_PID[0].max_out = CATCH_LEFT_MOUT;
    catch_PID[0].max_iout = CATCH_LEFT_MIOUT;
    catch_PID[0].Dbuf[0] = catch_PID[0].Dbuf[1] = catch_PID[0].Dbuf[2] = 0.0f;
    catch_PID[0].error[0] = catch_PID[0].error[1] = catch_PID[0].error[2] = catch_PID[0].Pout = catch_PID[0].Iout = catch_PID[0].Dout = catch_PID[0].out = 0.0f;

    catch_PID[1].mode = PID_POSITION;
    catch_PID[1].Kp = CATCH_RIGHT_KP;
    catch_PID[1].Ki = CATCH_RIGHT_KI;
    catch_PID[1].Kd = CATCH_RIGHT_KD;
    catch_PID[1].max_out = CATCH_RIGHT_MOUT;
    catch_PID[1].max_iout = CATCH_RIGHT_MIOUT;
    catch_PID[1].Dbuf[0] = catch_PID[1].Dbuf[1] = catch_PID[1].Dbuf[2] = 0.0f;
    catch_PID[1].error[0] = catch_PID[1].error[1] = catch_PID[1].error[2] = catch_PID[1].Pout = catch_PID[1].Iout = catch_PID[1].Dout = catch_PID[1].out = 0.0f;

    catch_PID[2].mode = PID_POSITION;
    catch_PID[2].Kp = CATCH_STRETCH_KP;
    catch_PID[2].Ki = CATCH_STRETCH_KI;
    catch_PID[2].Kd = CATCH_STRETCH_KD;
    catch_PID[2].max_out = CATCH_STRETCH_MOUT;
    catch_PID[2].max_iout = CATCH_STRETCH_MIOUT;
    catch_PID[2].Dbuf[0] = catch_PID[2].Dbuf[1] = catch_PID[2].Dbuf[2] = 0.0f;
    catch_PID[2].error[0] = catch_PID[2].error[1] = catch_PID[2].error[2] = catch_PID[2].Pout = catch_PID[2].Iout = catch_PID[2].Dout = catch_PID[2].out = 0.0f;

    catch_PID[3].mode = PID_POSITION;
    catch_PID[3].Kp = CATCH_CATCH_KP;
    catch_PID[3].Ki = CATCH_CATCH_KI;
    catch_PID[3].Kd = CATCH_CATCH_KD;
    catch_PID[3].max_out = CATCH_CATCH_MOUT;
    catch_PID[3].max_iout = CATCH_CATCH_MIOUT;
    catch_PID[3].Dbuf[0] = catch_PID[3].Dbuf[1] = catch_PID[3].Dbuf[2] = 0.0f;
    catch_PID[3].error[0] = catch_PID[3].error[1] = catch_PID[3].error[2] = catch_PID[3].Pout = catch_PID[3].Iout = catch_PID[3].Dout = catch_PID[3].out = 0.0f;

    catch_PID[4].mode = PID_POSITION;
    catch_PID[4].Kp = CATCH_ANGLE_KP;
    catch_PID[4].Ki = CATCH_ANGLE_KI;
    catch_PID[4].Kd = CATCH_ANGLE_KD;
    catch_PID[4].max_out = CATCH_ANGLE_MOUT;
    catch_PID[4].max_iout = CATCH_ANGLE_MIOUT;
    catch_PID[4].Dbuf[0] = catch_PID[4].Dbuf[1] = catch_PID[4].Dbuf[2] = 0.0f;
    catch_PID[4].error[0] = catch_PID[4].error[1] = catch_PID[4].error[2] = catch_PID[4].Pout = catch_PID[4].Iout = catch_PID[4].Dout = catch_PID[4].out = 0.0f;
}


//初始化函数指针及参数
void catch_init(void)
{

    catch.rc_data   =   get_remote_control_point();
    catch.auto_behave = get_auto_control_point();
    catch.reset_key = get_reset_point();

    catch_PID_init();

    for (uint8_t i = 0; i < 4; i++)
    {
        catch.motor_measure[i] = get_motor_measure_point(i);
    }
    catch.can.flip_state =   stop;
    catch.can.stretch_state =   stop;
    catch.can.catch_state =   close;
    catch.flip_angle = 0.0f;
    catch.stretch_lenth = 0.0f;
    catch_catch_flag = 0;
    catch.flip_stay_angle = catch.flip_angle;
    servo_data = 1500;
}

const catch_ctrl_t *get_catch_control_point(void)
{
    return &catch;
}
 