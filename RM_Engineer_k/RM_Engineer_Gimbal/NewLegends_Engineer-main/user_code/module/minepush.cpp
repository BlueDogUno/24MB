#include "minepush.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "tim.h"
/***     

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │
 *   │       ───       │++ + + +
 *   ███████───███████ │+
 *   │                 │+
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *              
 *               代码无BUG!
 */

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

int32_t STRETCH_LEN[AUTO_MODE_NUM] = {
        STRETCH_INIT_LEN,
        STRETCH_SKY_LEN,
        STRETCH_STANDARD_LEN,
        STRETCH_GROUND_LEN,
        STRETCH_DELIVERY_LEN,
    };

MinePush minepush;

PhotoSpin photospin;

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

void MinePush::init()
{
    mine_RC = remote_control.get_remote_control_point();
    last_mine_RC = remote_control.get_last_remote_control_point();
    photospin.sucker_flag = 0;

    for (uint8_t i = 0; i < 4; ++i)
    {

        //动力电机数据
        mine_motive_motor[i].init(can_receive.get_mine_motive_motor_measure_point(i));
        //初始化pid
        fp32 mine_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        mine_motive_motor[i].speed_pid.init(PID_SPEED, mine_speed_pid_parm, &mine_motive_motor[i].speed, &mine_motive_motor[i].speed_set, NULL);
        mine_motive_motor[i].speed_pid.pid_clear();

        fp32 mine_angle_pid_parm[5] = {MOTIVE_MOTOR_ANGLE_PID_KP, MOTIVE_MOTOR_ANGLE_PID_KI, MOTIVE_MOTOR_ANGLE_PID_KD, MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT, MOTIVE_MOTOR_ANGLE_PID_MAX_OUT};
        mine_motive_motor[i].angle_pid.init(PID_ANGLE, mine_angle_pid_parm, &mine_motive_motor[i].total_angle , &mine_motive_motor[i].angle_set, 0);
        mine_motive_motor[i].angle_pid.pid_clear();
    }
    vTaskDelay(1000);
    feedback_update();

    for (uint8_t i = 0; i < 4; ++i)
    {
        //设置初始值
        stretch_moto_start_angle[i] = mine_motive_motor[i].total_angle;
        mine_motive_motor[i].max_speed = NORMAL_MAX_STRETCH_SPEED;
        mine_motive_motor[i].min_speed = -NORMAL_MAX_STRETCH_SPEED;
        motor_status[i] = WAIT;
    }

    // 电机软件限位，需要测试后开启
    // mine_motive_motor[MINE_STRETCH_L_ID].max_angle = stretch_moto_start_angle[MINE_STRETCH_L_ID] + STRENTCH_LIMIT_ANGLE;
    // mine_motive_motor[MINE_STRETCH_L_ID].min_angle = stretch_moto_start_angle[MINE_STRETCH_L_ID];

    // mine_motive_motor[MINE_STRETCH_R_ID].max_angle = stretch_moto_start_angle[MINE_STRETCH_R_ID];
    // mine_motive_motor[MINE_STRETCH_R_ID].min_angle = stretch_moto_start_angle[MINE_STRETCH_R_ID] - STRENTCH_LIMIT_ANGLE;
    
    photospin.yaw_flag = 1;
    photospin.pitch_flag = 1;
    //更新一下数据
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    photospin.yaw_state = WALK_MODE;
    photospin.pitch_state = LOOK_MID;
}

/**
 * @brief          状态更新函数
 * @param[in]
 * @retval         none
 */
void MinePush::feedback_update(){
    //记录上一次遥控器值
    mine_last_key_v = mine_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度，加速度是速度的PID微分
        mine_motive_motor[i].speed = MINE_MOTOR_RPM_TO_VECTOR_SEN * mine_motive_motor[i].motor_measure->speed_rpm;
        mine_motive_motor[i].total_angle = mine_motive_motor[i].motor_measure->total_angle - stretch_moto_start_angle[i];
        mine_motive_motor[i].angle_error = mine_motive_motor[i].total_angle - mine_motive_motor[i].angle_set;
        if (mine_motive_motor[i].angle_error < ANGLE_ERR_TOLERANT && mine_motive_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
            motor_status[i] = READY;
        else
            motor_status[i] = WAIT;
    }
    
}

/**
 * @brief          行为切换设置
 * @param[in]
 * @retval         none
 */
void MinePush::set_mode(){
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"mine_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void MinePush::behaviour_mode_set()
{
    last_mine_behaviour_mode = mine_behaviour_mode;
    last_mine_mode = mine_mode;

    //遥控器设置模式
    if (switch_is_up(mine_RC->rc.s[STRETCH_MODE_CHANNEL])) //左拨杆上 开启push手动 无力底盘
    {
        mine_behaviour_mode = MINE_ZERO_FORCE;
    }
    else if (switch_is_mid(mine_RC->rc.s[STRETCH_MODE_CHANNEL])) //左拨杆中
    {
        mine_behaviour_mode =  MINE_OPEN;
    }
    else if (switch_is_down(mine_RC->rc.s[STRETCH_MODE_CHANNEL])) //左拨杆下
    {
        mine_behaviour_mode = MINE_ZERO_FORCE;
    }

    //根据行为模式选择一个控制模式
    if (mine_behaviour_mode == MINE_ZERO_FORCE || mine_behaviour_mode == MINE_OPEN)
    {
        mine_mode = MINE_HAND;
    }
    else if(mine_behaviour_mode == MINE_CLOSE)
    {
        mine_mode = MINE_AUTO;
    }



	if(if_key_pessed(mine_RC, '!') && if_key_pessed(mine_RC, 'Q'))//按q切换图传位置
    {
        photospin.yaw_flag = 0;
    }
    else if (if_key_pessed(mine_RC, '!') && if_key_pessed(mine_RC, 'E'))
    {
        photospin.yaw_flag = 1;
    }

	if (photospin.yaw_flag == 0)
	{
		photospin.yaw_state = EXCHANGE_MODE;
	}
    else
	{
	    photospin.yaw_state = WALK_MODE;
	}

    // if(if_key_pessed(mine_RC, 'R') && if_key_pessed(mine_RC, 'Q'))//按q切换图传位置
    // {
    //     photospin.pitch_flag ++ ;
    // }
    // else if (if_key_pessed(mine_RC, 'R') && if_key_pessed(mine_RC, 'E'))
    // {
    //     photospin.pitch_flag --;
    // }

	// if (photospin.pitch_flag == 0)
	// {
	// 	photospin.pitch_flag = EXCHANLOOK_UPGE_MODE;
	// }
    // else
	// {
	//     photospin.pitch_flag = WALK_MODE;
    if(if_key_pessed(mine_RC, '!') && if_key_pessed(mine_RC, 'F'))//开气泵
    {
        photospin.sucker_flag = 0;
    }
    else if (if_key_pessed(mine_RC, '!') && if_key_pessed(mine_RC, 'G'))
    {
        photospin.sucker_flag = 1;
    }
	// }



    if(if_key_pessed(mine_RC, '$') && if_key_pessed(mine_RC, 'Q'))
	{
		photospin.pitch_flag ++;
        if (photospin.pitch_flag > 2)
            photospin.pitch_flag = 2;
        if (photospin.pitch_flag < 0)
            photospin.pitch_flag = 0;
	}
	else if(if_key_pessed(mine_RC, '$') && if_key_pessed(mine_RC, 'E'))
	{
		photospin.pitch_flag --;
        if (photospin.pitch_flag > 2)
            photospin.pitch_flag = 2;
        if (photospin.pitch_flag < 0)
            photospin.pitch_flag = 0;
	}

    if(photospin.pitch_flag == 0)
    {
        photospin.pitch_state = LOOK_UP;
    }
    else if(photospin.pitch_flag == 1)
    {
        photospin.pitch_state = LOOK_MID;
    }
    else if(photospin.pitch_flag == 2)
    {
        photospin.pitch_state = LOOK_DOWN;
    }
        //遥控器设置图传舵机模式
    // if (switch_is_up(mine_RC->rc.s[PHOTO_MODE_CHANNEL])) //左拨杆上
    // {
    //     if (left_rocker_right){
    //         photospin.yaw_state = EXCHANGE_MODE; 
    //     }else if(left_rocker_left){
    //         photospin.yaw_state = WALK_MODE;
    //     }

    //     if (right_rocker_up){
    //         photospin.pitch_state = LOOK_UP;
    //     }else if (right_rocker_down){
    //         photospin.pitch_state = LOOK_DOWN;
    //     }

    // }
}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void MinePush::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制拨矿电机和伸爪电机
    //vmine_set控制拨矿电机速度，vstretch_set控制伸爪电机速度
    fp32 vmine_set = 0.0f, vstretch_set = 0.0f;
    fp32 angle_set = 0.0f;

    //获取控制设置值
    behaviour_control_set(&vmine_set, &vstretch_set);

    if (mine_mode == MINE_HAND)
    {
        mine_motive_motor[MINE_STRETCH_L_ID].angle_set += vstretch_set;
        mine_motive_motor[MINE_STRETCH_R_ID].angle_set += -1*vstretch_set;
    }
    
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vmine_set, 通常控制纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void MinePush::behaviour_control_set(fp32 *vmine_set, fp32 *vstretch_set)
{

    if (vmine_set == NULL || vstretch_set == NULL)
    {
        return;
    }
    //无力
    if (mine_behaviour_mode == MINE_ZERO_FORCE)
    {
        *vmine_set = 0.0f;
        *vstretch_set = 0.0f;
    }
    else if (mine_behaviour_mode == MINE_OPEN)
    {
        mine_open_set_control(vmine_set, vstretch_set);
    }

    last_mine_RC->key.v = mine_RC->key.v;
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void MinePush::mine_open_set_control(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    static int16_t stretch_channel = 0;

    rc_deadband_limit(mine_RC->rc.ch[MINE_X_CHANNEL], stretch_channel, RC_DEADBAND);
    *vy_set = mine_RC->rc.ch[MINE_X_CHANNEL] / MINE_OPEN_RC_SCALE;
}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void MinePush::solve()
{
    for (int i = 0; i < 4; i++)
    {
        motor_set_control(&mine_motive_motor[i]);
    }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void MinePush::output()
{
    if (mine_behaviour_mode == MINE_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            mine_motive_motor[i].current_give = 0.0f;
        }
    }
    // can_receive.can_cmd_mine_motive_motor(mine_motive_motor[MINE_STRETCH_L_ID].current_give, mine_motive_motor[MINE_STRETCH_R_ID].current_give,
    //                                       mine_motive_motor[MINE_PUSH_LEFT_ID].current_give, mine_motive_motor[MINE_PUSH_RIGHT_ID].current_give);
    can_receive.can_cmd_mine_motive_motor(0, 0,
                                          mine_motive_motor[MINE_STRETCH_L_ID].current_give, mine_motive_motor[MINE_STRETCH_R_ID].current_give);
    // can_receive.can_cmd_mine_motive_motor(0,0,0,0);
}

/**
 * @brief          双环pid计算
 * @param[out]     
 * @retval         none
 */

void MinePush::motor_set_control(Mine_motor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed_set = motor->angle_pid.pid_calc();
    if (motor->speed_set > motor->max_speed)
        motor->speed_set = motor->max_speed;
    if (motor->speed_set < motor->min_speed)
        motor->speed_set = motor->min_speed;
    motor->current_give = motor->speed_pid.pid_calc();
    
}

/**
 * @brief          伸爪结构自动控制
 * @param[out]     
 * @retval         none
 */
void MinePush::auto_control(auto_mode_e *auto_mode)
{
    switch(*auto_mode)
    {
        case CATCH_INIT:
        case CATCH_SKY:
        case CATCH_STANDARD:
        case CATCH_GROUND:
        case CATCH_DELIVERY:
        {
            static int AUTO_MODE;
            
            AUTO_MODE = *auto_mode - CATCH_INIT;
            mine_motive_motor[MINE_STRETCH_L_ID].angle_set = - STRETCH_LEN[AUTO_MODE];
            mine_motive_motor[MINE_STRETCH_R_ID].angle_set = + STRETCH_LEN[AUTO_MODE];
        }
    }
}

void PhotoSpin::output(){
    if(photospin.yaw_state == EXCHANGE_MODE)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, yaw_exchange_data);
    }
    else if (photospin.yaw_state == WALK_MODE)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, yaw_walk_data);
    }

    if(photospin.pitch_state == LOOK_UP){

		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pitch_look_up_data);

    }else if (photospin.pitch_state == LOOK_MID){

		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pitch_look_mid_data);

    }else if (photospin.pitch_state == LOOK_DOWN){

        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pitch_look_down_data);

    }

}

