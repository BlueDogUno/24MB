#include "lift.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "tim.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

uint8_t pump_state;

int32_t LIFT_LEN[AUTO_MODE_NUM] = {
        LIFT_INIT_LEN,
        LIFT_STANDARD_LEN,
        LIFT_GROUND_LEN,
        LIFT_DELIVERY_LEN
    };

Lift lift;

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


void Lift::init()
{
    lift_RC = remote_control.get_remote_control_point();
    last_lift_RC = remote_control.get_last_remote_control_point();
    pump_state = close;
    for (uint8_t i = 0; i < 2; ++i)
    {

        //动力电机数据
        lift_motive_motor[i].init(can_receive.get_chassis_lift_motor_measure_point(i));
        //初始化pid
        fp32 lift_speed_pid_parm[5] = {LIFT_F_MOTOR_SPEED_PID_KP, LIFT_F_MOTOR_SPEED_PID_KI, LIFT_F_MOTOR_SPEED_PID_KD, LIFT_F_MOTOR_SPEED_PID_MAX_IOUT, LIFT_F_MOTOR_SPEED_PID_MAX_OUT};
        lift_motive_motor[i].speed_pid.init(PID_SPEED, lift_speed_pid_parm, &lift_motive_motor[i].speed, &lift_motive_motor[i].speed_set, NULL);
        lift_motive_motor[i].speed_pid.pid_clear();

        fp32 lift_angle_pid_parm[5] = {MOTIVE_MOTOR_ANGLE_PID_KP, MOTIVE_MOTOR_ANGLE_PID_KI, MOTIVE_MOTOR_ANGLE_PID_KD, MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT, MOTIVE_MOTOR_ANGLE_PID_MAX_OUT};
        lift_motive_motor[i].angle_pid.init(PID_ANGLE, lift_angle_pid_parm, &lift_motive_motor[i].total_angle , &lift_motive_motor[i].angle_set, 0);
        lift_motive_motor[i].angle_pid.pid_clear();
        
        //设置初始值
        
        lift_motive_motor[i].max_speed = NORMAL_MAX_LIFT_SPEED;
        lift_motive_motor[i].min_speed = -NORMAL_MAX_LIFT_SPEED;

        motor_status[i] = WAIT;
    }
    // 电机软件限位，需要测试后开启
    
    vTaskDelay(1000);
    //更新一下数据
    feedback_update();

    motor_start_angle[CAN_LIFT_L_MOTOR] = lift_motive_motor[CAN_LIFT_L_MOTOR].total_angle;
    lift_motive_motor[CAN_LIFT_L_MOTOR].max_angle = + LIFT_LIMIT_ANGLE;
    lift_motive_motor[CAN_LIFT_L_MOTOR].min_angle = - LIFT_LIMIT_ANGLE;
    
    motor_start_angle[CAN_LIFT_R_MOTOR] = lift_motive_motor[CAN_LIFT_R_MOTOR].total_angle;
    lift_motive_motor[CAN_LIFT_R_MOTOR].max_angle = - LIFT_LIMIT_ANGLE;
    lift_motive_motor[CAN_LIFT_R_MOTOR].min_angle = + LIFT_LIMIT_ANGLE;

}

/**
 * @brief          状态更新函数
 * @param[in]
 * @retval         none
 */
void Lift::feedback_update(){
    //记录上一次遥控器值
    lift_last_key_v = lift_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 2; ++i)
    {
        //更新动力电机速度
        lift_motive_motor[i].speed = LIFT_MOTOR_RPM_TO_VECTOR_SEN * lift_motive_motor[i].motor_measure->speed_rpm;
        lift_motive_motor[i].total_angle = lift_motive_motor[i].motor_measure->total_angle - motor_start_angle[i];
        lift_motive_motor[i].angle_error = lift_motive_motor[i].total_angle - lift_motive_motor[i].angle_set;
        if (lift_motive_motor[i].angle_error < ANGLE_ERR_TOLERANT &&  lift_motive_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
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
void Lift::set_mode(){
    
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"lift_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Lift::behaviour_mode_set()
{
    last_lift_behaviour_mode = lift_behaviour_mode;
    last_lift_mode = lift_mode;

    //遥控器设置模式
    if (switch_is_mid(lift_RC->rc.s[LEFT_CHANNEL]) && switch_is_mid(lift_RC->rc.s[RIGHT_CHANNEL])) //左拨杆中 右拨杆中  minepush时可以抬升
    {
        lift_behaviour_mode = LIFT_OPEN;
    }
    else 
    {
        lift_behaviour_mode = LIFT_HOLD;
    }
 
    //根据行为模式选择一个控制模式
    if (lift_behaviour_mode == LIFT_ZERO_FORCE || lift_behaviour_mode == LIFT_OPEN)
    {
        lift_mode = LIFT_HAND;
    }
    else if(lift_behaviour_mode == LIFT_CLOSE)
    {
        lift_mode = LIFT_AUTO;
    }else if(lift_behaviour_mode == LIFT_HOLD){

        lift_mode = LIFT_HOLDM;
    }

    //键鼠控制
    // if (if_key_singal_pessed(lift_RC , last_lift_RC , KEY_PRESSED_PUMP_STATE))
    // {
    //     if (pump_state == open)
    //         pump_state = close;
    //     else    
    //         pump_state = open;
    // }
    //吸盘开关
    // if(if_key_pessed(lift_RC, '!') && if_key_pessed(lift_RC, 'F'))//开气泵
    // {
    //     pump_state = close;
    // }
    // else if (if_key_pessed(lift_RC, '!') && if_key_pessed(lift_RC, 'G'))
    // {
    //     pump_state = open;
    // }

}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Lift::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制拨矿电机和伸爪电机
    //vz_set控制抬升电机速度，vstretch_set控制电机速度, 
    fp32 vz_set = 0.0f;

    //获取控制设置值
    behaviour_control_set(&vz_set);

    if (lift_mode == LIFT_HAND)
    {
        //同轴有一个是相反的
        lift_motive_motor[CAN_LIFT_L_MOTOR].angle_set += vz_set;
        lift_motive_motor[CAN_LIFT_R_MOTOR].angle_set -= vz_set;
        angle_hold_set = lift_motive_motor[CAN_LIFT_L_MOTOR].angle_set;
    }
    if (lift_mode == LIFT_HOLD){
        lift_motive_motor[CAN_LIFT_L_MOTOR].angle_set = angle_hold_set;
        lift_motive_motor[CAN_LIFT_R_MOTOR].angle_set = -1*angle_hold_set;
    }
    // 做角度限幅
    for (int i = 0;i < 2;i++)
    {
        // motor_angle_limit(&lift_motive_motor[i]);
    }


}



/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vz_set, 通常翻转机构纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void Lift::behaviour_control_set(fp32 *vz_set)
{
    if (vz_set == NULL)
    {
        return;
    }
    //无力
    if (lift_behaviour_mode == LIFT_ZERO_FORCE)
    {
        *vz_set = 0.0f;
    }
    else if (lift_behaviour_mode == LIFT_OPEN)
    {
        lift_open_set_control(vz_set);

    }

    last_lift_RC->key.v = lift_RC->key.v;
}

/**
 * @brief          抬升状态控制
 * @param[in]      vz_set夹爪翻转的速度
 * @param[in]      数据
 * @retval         none
 */
void Lift::lift_open_set_control(fp32 *vz_set)
{
    if (vz_set == NULL )
    {
        return;
    }
    static int16_t lift_channel = 0;

    rc_deadband_limit(lift_RC->rc.ch[LIFT_X_CHANNEL], lift_channel, RC_DEADBAND);
    *vz_set = lift_RC->rc.ch[LIFT_X_CHANNEL] / LIFT_OPEN_RC_SCALE;

}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Lift::solve()
{
    if (lift_behaviour_mode == LIFT_OPEN || lift_behaviour_mode == LIFT_HOLD || lift_behaviour_mode == LIFT_CLOSE)
        for (int i = 0; i < 2; i++)
        {
            motor_set_control(&lift_motive_motor[i]);
        }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Lift::output()
{
     for (int i = 0; i < 2; i++)
    {
        lift_motive_motor[i].current_give = (int16_t)lift_motive_motor[i].current_set;
    }
    if (lift_behaviour_mode == LIFT_ZERO_FORCE)
    {
        for (int i = 0; i < 2; i++)
        {
            lift_motive_motor[i].current_give = 0.0f;
        }
    }

    can_receive.can_cmd_chassis_lift_motor(lift_motive_motor[CAN_LIFT_L_MOTOR].current_give, lift_motive_motor[CAN_LIFT_R_MOTOR].current_give);
    // can_receive.can_cmd_chassis_lift_motor(0, 0);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */

void Lift::motor_set_control(M3508_motor *motor)
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
    motor->current_set = motor->speed_pid.pid_calc();
    
}

void Lift::motor_angle_limit(M3508_motor *motor)
{
    if (motor->total_angle < motor->min_angle)
    {
        motor->total_angle = motor->min_angle;
    }
    else if (motor->total_angle > motor->max_angle)
    {
        motor->total_angle = motor->max_angle;
    }
}


/**
 * @brief          自动模式控制电机转动角度
 * @param[out]     add: 角度增加量
 * @retval         none
 */
void Lift::auto_control(auto_mode_e *auto_mode)
{
    switch(*auto_mode)
    {
        case LIFT_INIT:
        case LIFT_SKY:
        case LIFT_STANDARD:
        case LIFT_GROUND:
        case LIFT_DELIVERY:
        {
            static int AUTO_MODE;
            AUTO_MODE = *auto_mode - LIFT_INIT;
            lift_motive_motor[CAN_LIFT_L_MOTOR].angle_set = + LIFT_LEN[AUTO_MODE];
            lift_motive_motor[CAN_LIFT_R_MOTOR].angle_set = - LIFT_LEN[AUTO_MODE];
        }
    }
}

//================================================================

void Lift::pump_contorl_send(){
        if (pump_state == open)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
        }
        else if (pump_state == close)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); 
        } 

}
