/*
此处代码为 臂 的手动控制
写在一处 方便之后 的路径规划的编写
author:youg
*/

#include "arm_ctrl.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "tim.h"
#include "Remote_control.h"
#include "DMPower.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

//定义小臂模块  对象
Arm_ctrl arm_ctrl;

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

/**
 * @brief          初始化变量，包括pid初始化， 遥控器指针初始化等
 * @param[out]
 * @retval         none
 */
void Arm_ctrl::init()
{
    arm_RC = remote_control.get_remote_control_point();
    last_arm_RC = remote_control.get_last_remote_control_point();

    for (uint8_t i = 0; i < 4; ++i)
    {
        //获取终端动力电机数据
        terminal_motor[i].init(can_receive.get_terminal_motor_measure_point(i));
        //获取肘部动力电机数据
        elbow_motor[i].init(can_receive.get_elbow_motor_measure_point(i));

        //设置初始值  速度限幅
        terminal_motor[i].max_speed = NORMAL_MAX_ARM_SPEED;
        terminal_motor[i].min_speed = -NORMAL_MAX_ARM_SPEED;
        elbow_motor[i].max_speed = NORMAL_MAX_ARM_SPEED;
        elbow_motor[i].min_speed = -NORMAL_MAX_ARM_SPEED;

        //电机状态初始化
        terminal_motor_status[i] = WAIT;
        elbow_motor_status[i] = WAIT;
    }

    //初始化roll轴pid
    fp32 roll_speed_pid_parm[5] = {ROLL_MOTOR_SPEED_PID_KP, ROLL_MOTOR_SPEED_PID_KI, ROLL_MOTOR_SPEED_PID_KD, ROLL_MOTOR_SPEED_PID_MAX_IOUT, ROLL_MOTOR_SPEED_PID_MAX_OUT};
    terminal_motor[0].speed_pid.init(PID_SPEED, roll_speed_pid_parm, &terminal_motor[3].speed, &terminal_motor[3].speed_set, NULL);
    terminal_motor[0].speed_pid.pid_clear();
    fp32 roll_angle_pid_parm[5] = {ROLL_MOTOR_ANGLE_PID_KP, ROLL_MOTOR_ANGLE_PID_KI, ROLL_MOTOR_ANGLE_PID_KD, ROLL_MOTOR_ANGLE_PID_MAX_IOUT, ROLL_MOTOR_ANGLE_PID_MAX_OUT};
    terminal_motor[0].angle_pid.init(PID_ANGLE, roll_angle_pid_parm, &terminal_motor[3].total_angle , &terminal_motor[3].angle_set, 0);
    terminal_motor[0].angle_pid.pid_clear();

    //初始化yaw轴pid
    fp32 yaw_speed_pid_parm[5] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD, YAW_MOTOR_SPEED_PID_MAX_IOUT, YAW_MOTOR_SPEED_PID_MAX_OUT};
    terminal_motor[0].speed_pid.init(PID_SPEED, yaw_speed_pid_parm, &terminal_motor[3].speed, &terminal_motor[3].speed_set, NULL);
    terminal_motor[0].speed_pid.pid_clear();
    fp32 yaw_angle_pid_parm[5] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD, YAW_MOTOR_ANGLE_PID_MAX_IOUT, YAW_MOTOR_ANGLE_PID_MAX_OUT};
    terminal_motor[0].angle_pid.init(PID_ANGLE, yaw_angle_pid_parm, &terminal_motor[3].total_angle , &terminal_motor[3].angle_set, 0);
    terminal_motor[0].angle_pid.pid_clear();

    //初始化pitch轴pid
    fp32 pitch_speed_pid_parm[5] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD, PITCH_MOTOR_SPEED_PID_MAX_IOUT, PITCH_MOTOR_SPEED_PID_MAX_OUT};
    terminal_motor[0].speed_pid.init(PID_SPEED, pitch_speed_pid_parm, &terminal_motor[3].speed, &terminal_motor[3].speed_set, NULL);
    terminal_motor[0].speed_pid.pid_clear();
    fp32 pitch_angle_pid_parm[5] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD, PITCH_MOTOR_ANGLE_PID_MAX_IOUT, PITCH_MOTOR_ANGLE_PID_MAX_OUT};
    terminal_motor[0].angle_pid.init(PID_ANGLE, pitch_angle_pid_parm, &terminal_motor[3].total_angle , &terminal_motor[3].angle_set, 0);
    terminal_motor[0].angle_pid.pid_clear();

    //初始化x_stretch轴pid
    fp32 x_stretch_speed_pid_parm[5] = {X_STRETCH_MOTOR_SPEED_PID_KP, X_STRETCH_MOTOR_SPEED_PID_KI, X_STRETCH_MOTOR_SPEED_PID_KD, X_STRETCH_MOTOR_SPEED_PID_MAX_IOUT, X_STRETCH_MOTOR_SPEED_PID_MAX_OUT};
    elbow_motor[2].speed_pid.init(PID_SPEED, x_stretch_speed_pid_parm, &elbow_motor[3].speed, &elbow_motor[3].speed_set, NULL);
    elbow_motor[2].speed_pid.pid_clear();

     //初始化y_slid轴pid
    fp32 y_slid_speed_pid_parm[5] = {Y_SLID_MOTOR_SPEED_PID_KP, Y_SLID_MOTOR_SPEED_PID_KI, Y_SLID_MOTOR_SPEED_PID_KD, Y_SLID_MOTOR_SPEED_PID_MAX_IOUT, Y_SLID_MOTOR_SPEED_PID_MAX_OUT};
    elbow_motor[1].speed_pid.init(PID_SPEED, y_slid_speed_pid_parm, &elbow_motor[3].speed, &elbow_motor[3].speed_set, NULL);
    elbow_motor[1].speed_pid.pid_clear();
    
    vTaskDelay(1000);
    //更新一下数据
    feedback_update();
    
}

/**
 * @brief          行为切换设置
 * @param[in]
 * @retval         none
 */
void Arm_ctrl::set_mode()
{
    behaviour_mode_set();
}


/**
 * @brief          小臂测量数据更新，包括电机速度，角度
 * @param[in]
 * @retval         none
 */
void Arm_ctrl::feedback_update(){
    //记录上一次遥控器值
    arm_last_key_v = arm_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新电机速度
        terminal_motor[i].speed = ARM_MOTOR_RPM_TO_VECTOR_SEN * terminal_motor[i].motor_measure->speed_rpm;
        elbow_motor[i].speed = ARM_MOTOR_RPM_TO_VECTOR_SEN * elbow_motor[i].motor_measure->speed_rpm;
        //更新电机角度
        terminal_motor[i].total_angle = terminal_motor[i].motor_measure->total_angle - terminal_motor_start_angle[i];
        terminal_motor[i].angle_error = terminal_motor[i].total_angle - terminal_motor[i].angle_set;
        elbow_motor[i].total_angle = elbow_motor[i].motor_measure->total_angle - elbow_motor_start_angle[i];
        elbow_motor[i].angle_error = elbow_motor[i].total_angle - elbow_motor[i].angle_set;
        //误差区间
        if (terminal_motor[i].angle_error < ANGLE_ERR_TOLERANT &&  terminal_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
        {
            terminal_motor_status[i] = READY;
        }   
        else
        {
            terminal_motor_status[i] = WAIT;
        }
        if (elbow_motor[i].angle_error < ANGLE_ERR_TOLERANT &&  elbow_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
        {
            elbow_motor_status[i] = READY;
        }   
        else
        {
            elbow_motor_status[i] = WAIT;
        }
    }
}



/**
 * @brief          通过逻辑判断，赋值"arm_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Arm_ctrl::behaviour_mode_set()
{
    last_arm_behaviour_mode = arm_behaviour_mode;
    last_arm_mode = arm_mode;

    //遥控器设置模式
    //左拨杆上 末端控制模式
    if (switch_is_down(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]) && switch_is_down(arm_RC->rc.s[MODE_SWITCH_CHANNEL]))
    {
        //下下手臂无力
        arm_behaviour_mode = ARM_ZERO_FORCE;
    }
    else if(switch_is_up(arm_RC->rc.s[MODE_SWITCH_CHANNEL]))
    {
        
        if (switch_is_up(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]))
        {
            //控制yaw轴
            arm_behaviour_mode = ARM_YAW; 
        }
        else if (switch_is_mid(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]))
        {
            //控制pitch轴
            arm_behaviour_mode = ARM_PITCH;
        }
        else if(switch_is_down(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]))
        {
            //控制roll轴
            arm_behaviour_mode = ARM_ROLL;
        }
        else 
        {
            arm_behaviour_mode = ARM_HOLD;
        }
    } //左拨杆中 肘部控制模式
    else if (switch_is_mid(arm_RC->rc.s[MODE_SWITCH_CHANNEL]))
    {
        if (switch_is_up(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]))
        {
            //控制x_stretch轴
            arm_behaviour_mode = ARM_X_STRETCH;
        }
        else if(switch_is_down(arm_RC->rc.s[JOINT_CHOOSE_CHANNEL]))
        {
            //控制y_slid轴
            arm_behaviour_mode = ARM_Y_SLID;
        }
        else 
        {
            arm_behaviour_mode = ARM_HOLD;
        }
    } 
    else 
    {
        arm_behaviour_mode = ARM_HOLD;
    }

    //根据行为模式选择一个控制模式

    if(arm_behaviour_mode == ARM_CLOSE)
    {
        arm_mode = ARM_HAND;//ARM_AUTO
    } 
    else if(arm_behaviour_mode == ARM_HOLD)
    {
        arm_mode = ARM_HOLD_MODE;
    } 
    else
    {
        arm_mode = ARM_HAND;
    }
}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Arm_ctrl::set_control()
{
    //所有关节的v_set 速度设定值
    fp32 vyaw_set,vpitch_set,vroll_set;
    fp32 vx_set,vy_set;

    //获取控制设置值
    behaviour_control_set(&vyaw_set,&vpitch_set,&vroll_set,&vx_set,&vy_set);

    if (arm_mode == ARM_HAND)
    {
        if (arm_behaviour_mode == ARM_YAW)
        {
            terminal_motor[CAN_TERMINAL_YAW].angle_set += vyaw_set;
        }
        else if (arm_behaviour_mode == ARM_PITCH)
        {
            terminal_motor[CAN_TERMINAL_PITCH].angle_set += vpitch_set;
        }
        else if (arm_behaviour_mode == ARM_ROLL)
        {
            terminal_motor[CAN_TERMINAL_ROLL].angle_set += vroll_set;
        }
        else if (arm_behaviour_mode == ARM_X_STRETCH)
        {
            elbow_motor[CAN_ELBOW_X_STRETCH].angle_set += vx_set;
        }
        else if (arm_behaviour_mode == ARM_Y_SLID)
        {
            elbow_motor[CAN_ELBOW_Y_SLID].angle_set += vy_set;
        }

        terminal_hold_angle[CAN_TERMINAL_YAW] = terminal_motor[CAN_TERMINAL_YAW].angle_set;
        terminal_hold_angle[CAN_TERMINAL_PITCH] = terminal_motor[CAN_TERMINAL_PITCH].angle_set;
        terminal_hold_angle[CAN_TERMINAL_ROLL] = terminal_motor[CAN_TERMINAL_ROLL].angle_set;

    }
    if(arm_mode == ARM_HOLD_MODE)
    {
        terminal_motor[CAN_TERMINAL_YAW].angle_set = terminal_hold_angle[CAN_TERMINAL_YAW];
        terminal_motor[CAN_TERMINAL_PITCH].angle_set = terminal_hold_angle[CAN_TERMINAL_PITCH];
        terminal_motor[CAN_TERMINAL_ROLL].angle_set = terminal_hold_angle[CAN_TERMINAL_ROLL];

    }

}



/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vforearm_set, 通常翻转机构纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void Arm_ctrl::behaviour_control_set(fp32 *vyaw_set, fp32 *vpitch_set,fp32 *vroll_set,fp32 *vx_set,fp32 *vy_set)
{
    if (vyaw_set == NULL || vpitch_set == NULL || vroll_set == NULL|| vx_set == NULL|| vy_set == NULL)
    {
        return;
    }
    
    if (arm_behaviour_mode == ARM_ZERO_FORCE)//无力
    {
        *vyaw_set = 0.0f;
        *vpitch_set = 0.0f;
        *vroll_set = 0.0f;
        *vx_set = 0.0f;
        *vy_set = 0.0f;
    }
    else
    {
        arm_open_set_control(vyaw_set, vpitch_set, vroll_set, vx_set, vy_set);
    }

    last_arm_RC->key.v = arm_RC->key.v;
}



/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set夹爪翻转的速度
 * @param[in]      vy_set抓取机构YAW轴的速度
 * @param[in]      wz_set吸盘旋转速度
 * @param[in]      数据
 * @retval         none
 */
void Arm_ctrl::arm_open_set_control(fp32 *vyaw_set, fp32 *vpitch_set,fp32 *vroll_set,fp32 *vx_set,fp32 *vy_set)
{
    if (vyaw_set == NULL || vpitch_set == NULL || vroll_set == NULL|| vx_set == NULL|| vy_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0, roll_channel = 0,x_channel = 0, y_channel = 0;

    rc_deadband_limit(arm_RC->rc.ch[ARM_CTRL_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(arm_RC->rc.ch[ARM_CTRL_CHANNEL], pitch_channel, RC_DEADBAND);
    rc_deadband_limit(arm_RC->rc.ch[ARM_CTRL_CHANNEL], roll_channel, RC_DEADBAND);
    rc_deadband_limit(arm_RC->rc.ch[ARM_CTRL_CHANNEL], x_channel, RC_DEADBAND);
    rc_deadband_limit(arm_RC->rc.ch[ARM_CTRL_CHANNEL], y_channel, RC_DEADBAND);

    *vx_set = x_channel / ARM_OPEN_RC_SCALE;
    *vy_set = y_channel/ ARM_OPEN_RC_SCALE;
    *vyaw_set = yaw_channel / ARM_OPEN_RC_SCALE;
    *vpitch_set = pitch_channel / ARM_OPEN_RC_SCALE;
    *vroll_set = roll_channel / ARM_OPEN_RC_SCALE;
    
}



/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Arm_ctrl::solve()
{      
    for (int i = 0; i < 4; i++)
    {
        motor_set_control(&terminal_motor[i]);
        motor_set_control(&elbow_motor[i]);
    }

}



/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Arm_ctrl::output()
{
    // if (arm_behaviour_mode == ARM_ZERO_FORCE)
    // {
    //     for (int i = 0; i < 4; i++)
    //     {
    //         terminal_motor[i].current_give = 0.0f;
    //         elbow_motor[i].current_give = 0.0f;
    //     }
    // }
    // can_receive.can_cmd_arm_motor(terminal_motor[CAN_TERMINAL_ROLL].current_give, elbow_motor[CAN_ELBOWY_SLID].current_give,
                                        //  elbow_motor[CAN_ELBOWX_STRETCH].current_give, 0);
}



/**
 * @brief          跑角度环
 * @param[out]     Joint_motor:关节电机
 * @retval         none
 */
void Arm_ctrl::motor_set_control(Joint_motor *motor)
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
 * @brief          角度限制
 * @param[out]     Joint_motor:关节电机
 * @retval         none
 */
void Arm_ctrl::motor_angle_limit(Joint_motor *motor)
{
    if (motor->angle_set < motor->min_angle)
    {
        motor->angle_set = motor->min_angle;
    }
    else if (motor->angle_set > motor->max_angle)
    {
        motor->angle_set = motor->max_angle;
    }
}
