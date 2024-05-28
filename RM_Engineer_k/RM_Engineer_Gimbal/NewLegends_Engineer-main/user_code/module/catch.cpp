
#include "catch.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
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

int32_t ROTATE_ANGLE[AUTO_MODE_NUM][CAN_CATCH_SUCTION_MOTOR] = {
        {CATCH_INIT_SPIN_ANGLE, CATCH_INIT_YAW_ANGLE, CATCH_INIT_SUCTION_ANGLE},
        {CATCH_SKY_SPIN_ANGLE, CATCH_SKY_YAW_ANGLE, CATCH_SKY_SUCTION_ANGLE},
        {CATCH_STANDARD_SPIN_ANGLE, CATCH_STANDARD_YAW_ANGLE, CATCH_STANDARD_SUCTION_ANGLE},
        {CATCH_GROUND_SPIN_ANGLE, CATCH_GROUND_YAW_ANGLE, CATCH_GROUND_SUCTION_ANGLE},
        {CATCH_DELIVERY_SPIN_ANGLE, CATCH_DELIVERY_YAW_ANGLE, CATCH_DELIVERY_SUCTION_ANGLE}
    };

Catch minecatch;

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

void Catch::init()
{
    catch_RC = remote_control.get_remote_control_point();
    last_catch_RC = remote_control.get_last_remote_control_point();

    for (uint8_t i = 0; i < 4; ++i)
    {
        //动力电机数据
        catch_motive_motor[i].init(can_receive.get_catch_motive_motor_measure_point(i));
        //初始化pid
        
        //设置初始值
        
        catch_motive_motor[i].max_speed = NORMAL_MAX_CATCH_SPEED;
        catch_motive_motor[i].min_speed = -NORMAL_MAX_CATCH_SPEED;

        motor_status[i] = WAIT;
    }

    //翻爪左
    fp32 flip_l_speed_pid_parm[5] = {FLIP_L_MOTOR_SPEED_PID_KP, FLIP_L_MOTOR_SPEED_PID_KI, FLIP_L_MOTOR_SPEED_PID_KD, FLIP_L_MOTOR_SPEED_PID_MAX_IOUT, FLIP_L_MOTOR_SPEED_PID_MAX_OUT};
    catch_motive_motor[0].speed_pid.init(PID_SPEED, flip_l_speed_pid_parm, &catch_motive_motor[0].speed, &catch_motive_motor[0].speed_set, NULL);
    catch_motive_motor[0].speed_pid.pid_clear();

    fp32 flip_l_angle_pid_parm[5] = {FLIP_L_MOTOR_ANGLE_PID_KP, FLIP_L_MOTOR_ANGLE_PID_KI, FLIP_L_MOTOR_ANGLE_PID_KD, FLIP_L_MOTOR_ANGLE_PID_MAX_IOUT, FLIP_L_MOTOR_ANGLE_PID_MAX_OUT};
    catch_motive_motor[0].angle_pid.init(PID_ANGLE, flip_l_angle_pid_parm, &catch_motive_motor[0].total_angle , &catch_motive_motor[0].angle_set, 0);
    catch_motive_motor[0].angle_pid.pid_clear();
    //翻爪右
    fp32 flip_r_speed_pid_parm[5] = {FLIP_R_MOTOR_SPEED_PID_KP, FLIP_R_MOTOR_SPEED_PID_KI, FLIP_R_MOTOR_SPEED_PID_KD, FLIP_R_MOTOR_SPEED_PID_MAX_IOUT, FLIP_R_MOTOR_SPEED_PID_MAX_OUT};
    catch_motive_motor[1].speed_pid.init(PID_SPEED, flip_r_speed_pid_parm, &catch_motive_motor[1].speed, &catch_motive_motor[1].speed_set, NULL);
    catch_motive_motor[1].speed_pid.pid_clear();

    fp32 flip_r_angle_pid_parm[5] = {FLIP_R_MOTOR_ANGLE_PID_KP, FLIP_R_MOTOR_ANGLE_PID_KI, FLIP_R_MOTOR_ANGLE_PID_KD, FLIP_R_MOTOR_ANGLE_PID_MAX_IOUT, FLIP_R_MOTOR_ANGLE_PID_MAX_OUT};
    catch_motive_motor[1].angle_pid.init(PID_ANGLE, flip_r_angle_pid_parm, &catch_motive_motor[1].total_angle , &catch_motive_motor[1].angle_set, 0);
    catch_motive_motor[1].angle_pid.pid_clear();

    //yaw轴
    fp32 yaw_speed_pid_parm[5] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD, YAW_MOTOR_SPEED_PID_MAX_IOUT, YAW_MOTOR_SPEED_PID_MAX_OUT};
    catch_motive_motor[2].speed_pid.init(PID_SPEED, yaw_speed_pid_parm, &catch_motive_motor[2].speed, &catch_motive_motor[2].speed_set, NULL);
    catch_motive_motor[2].speed_pid.pid_clear();

    fp32 yaw_angle_pid_parm[5] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD, YAW_MOTOR_ANGLE_PID_MAX_IOUT, YAW_MOTOR_ANGLE_PID_MAX_OUT};
    catch_motive_motor[2].angle_pid.init(PID_ANGLE, yaw_angle_pid_parm, &catch_motive_motor[2].total_angle , &catch_motive_motor[2].angle_set, 0);
    catch_motive_motor[2].angle_pid.pid_clear();

    //roll轴
    fp32 roll_speed_pid_parm[5] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD, PITCH_MOTOR_SPEED_PID_MAX_IOUT, PITCH_MOTOR_SPEED_PID_MAX_OUT};
    catch_motive_motor[3].speed_pid.init(PID_SPEED, roll_speed_pid_parm, &catch_motive_motor[3].speed, &catch_motive_motor[3].speed_set, NULL);
    catch_motive_motor[3].speed_pid.pid_clear();

    fp32 roll_angle_pid_parm[5] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD, PITCH_MOTOR_ANGLE_PID_MAX_IOUT, PITCH_MOTOR_ANGLE_PID_MAX_OUT};
    catch_motive_motor[3].angle_pid.init(PID_ANGLE, roll_angle_pid_parm, &catch_motive_motor[3].total_angle , &catch_motive_motor[3].angle_set, 0);
    catch_motive_motor[3].angle_pid.pid_clear();

    // 电机软件限位，需要测试后开启
    
    vTaskDelay(1000);
    //更新一下数据
    feedback_update();
    moto_start_angle[CAN_CATCH_YAW_MOTOR] = catch_motive_motor[CAN_CATCH_YAW_MOTOR].total_angle;
    catch_motive_motor[CAN_CATCH_YAW_MOTOR].max_angle =  + YAW_LIMIT_ANGLE;
    catch_motive_motor[CAN_CATCH_YAW_MOTOR].min_angle =  - YAW_LIMIT_ANGLE;
    
    moto_start_angle[CAN_CATCH_SUCTION_MOTOR] = catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].total_angle;
    catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].max_angle =  + SUCTION_LIMIT_ANGLE;
    catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].min_angle =  - SUCTION_LIMIT_ANGLE;

    moto_start_angle[CAN_SPIN_L_MOTOR] = catch_motive_motor[CAN_SPIN_L_MOTOR].total_angle;
    catch_motive_motor[CAN_SPIN_L_MOTOR].max_angle = + SPIN_LIMIT_ANGLE;
    catch_motive_motor[CAN_SPIN_L_MOTOR].min_angle =  - SPIN_LIMIT_ANGLE;

    moto_start_angle[CAN_SPIN_R_MOTOR] = catch_motive_motor[CAN_SPIN_R_MOTOR].total_angle;
    catch_motive_motor[CAN_SPIN_R_MOTOR].max_angle = + SPIN_LIMIT_ANGLE;
    catch_motive_motor[CAN_SPIN_R_MOTOR].min_angle = - SPIN_LIMIT_ANGLE;
}

/**
 * @brief          状态更新函数
 * @param[in]
 * @retval         none
 */
void Catch::feedback_update(){
    //记录上一次遥控器值
    catch_last_key_v = catch_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度
        catch_motive_motor[i].speed = CATCH_MOTOR_RPM_TO_VECTOR_SEN * catch_motive_motor[i].motor_measure->speed_rpm;
        catch_motive_motor[i].total_angle = catch_motive_motor[i].motor_measure->total_angle - moto_start_angle[i];
        catch_motive_motor[i].angle_error = catch_motive_motor[i].total_angle - catch_motive_motor[i].angle_set;
        if (catch_motive_motor[i].angle_error < ANGLE_ERR_TOLERANT &&  catch_motive_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
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
void Catch::set_mode(){
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"catch_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Catch::behaviour_mode_set()
{
    last_catch_behaviour_mode = catch_behaviour_mode;
    last_catch_mode = catch_mode;

    //遥控器设置模式
    if (switch_is_up(catch_RC->rc.s[CATCH_MODE_CHANNEL])) //右拨杆上 
    {
        catch_behaviour_mode = CATCH_OPEN;
    }
    else if (switch_is_mid(catch_RC->rc.s[CATCH_MODE_CHANNEL])) //右拨杆中  开启catch手动
    {
        catch_behaviour_mode = CATCH_HOLD;
    }
    else if (switch_is_down(catch_RC->rc.s[CATCH_MODE_CHANNEL])) //右拨杆下
    {
        catch_behaviour_mode = CATCH_HOLD;
    }
    if (switch_is_down(catch_RC->rc.s[0])) //左拨杆下
    {
        catch_behaviour_mode = CATCH_ZERO_FORCE;
    }


    //根据行为模式选择一个控制模式
    if (catch_behaviour_mode == CATCH_ZERO_FORCE || catch_behaviour_mode == CATCH_OPEN)
    {
        catch_mode = CATCH_HAND;
    }
    else if(catch_behaviour_mode == CATCH_CLOSE)
    {
        catch_mode = CATCH_AUTO;
    }
    else if(catch_behaviour_mode == CATCH_HOLD)
    {

        catch_mode = CATCH_HOLDM;
    }

}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Catch::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制拨矿电机和伸爪电机
    //vspin_set控制电机速度，vyaw_set控制电机速度, 
    fp32 vspin_set = 0.0f, vyaw_set = 0.0f, vsuction_set = 0.0f;

    //获取控制设置值
    behaviour_control_set(&vspin_set, &vyaw_set, &vsuction_set);

    if (catch_mode == CATCH_HAND)
    {
        //同轴有一个是相反的
        catch_motive_motor[CAN_SPIN_L_MOTOR].angle_set += vspin_set;
        catch_motive_motor[CAN_SPIN_R_MOTOR].angle_set += -vspin_set;
        catch_motive_motor[CAN_CATCH_YAW_MOTOR].angle_set += vyaw_set;
        catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].angle_set += vsuction_set;

        catch_hold_angle[CAN_SPIN_L_MOTOR] = catch_motive_motor[CAN_SPIN_L_MOTOR].angle_set;
		catch_hold_angle[CAN_SPIN_R_MOTOR] = catch_motive_motor[CAN_SPIN_R_MOTOR].angle_set;
        catch_hold_angle[CAN_CATCH_YAW_MOTOR] = catch_motive_motor[CAN_CATCH_YAW_MOTOR].angle_set;
        catch_hold_angle[CAN_CATCH_SUCTION_MOTOR] = catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].angle_set;
    }
    if(catch_mode == CATCH_HOLDM)
    {
        catch_motive_motor[CAN_SPIN_L_MOTOR].angle_set =  catch_hold_angle[CAN_SPIN_L_MOTOR];
        catch_motive_motor[CAN_SPIN_R_MOTOR].angle_set =  catch_hold_angle[CAN_SPIN_R_MOTOR];
        catch_motive_motor[CAN_CATCH_YAW_MOTOR].angle_set =  catch_hold_angle[CAN_CATCH_YAW_MOTOR];
        catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].angle_set =  catch_hold_angle[CAN_CATCH_SUCTION_MOTOR];

    }
    // 做角度限幅
    for (int i = 0;i < 4;i++)
    {
        motor_angle_limit(&catch_motive_motor[i]);
    }

}



/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vcatch_set, 通常翻转机构纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void Catch::behaviour_control_set(fp32 *vcatch_set, fp32 *vyaw_set, fp32 *vsuction_set)
{
    if (vcatch_set == NULL || vyaw_set == NULL || vsuction_set == NULL)
    {
        return;
    }
    //无力
    if (catch_behaviour_mode == CATCH_ZERO_FORCE)
    {
        *vcatch_set = 0.0f;
        *vyaw_set = 0.0f;
        *vsuction_set = 0.0f;
    }
    else
    {
        catch_open_set_control(vcatch_set, vyaw_set, vsuction_set);

    }

    last_catch_RC->key.v = catch_RC->key.v;
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set夹爪翻转的速度
 * @param[in]      vy_set抓取机构YAW轴的速度
 * @param[in]      wz_set吸盘旋转速度
 * @param[in]      数据
 * @retval         none
 */
void Catch::catch_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set)
{
    if (vx_set == NULL || vy_set == NULL || vz_set == NULL)
    {
        return;
    }
    static int16_t catch_channel = 0, yaw_channel = 0, suction_channel = 0;

    rc_deadband_limit(catch_RC->rc.ch[CATCH_X_CHANNEL], catch_channel, RC_DEADBAND);
    rc_deadband_limit(catch_RC->rc.ch[CATCH_Y_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(catch_RC->rc.ch[CATCH_Z_CHANNEL], suction_channel, RC_DEADBAND);

    *vx_set = catch_RC->rc.ch[CATCH_X_CHANNEL] / CATCH_OPEN_RC_SCALE;
    *vy_set = -catch_RC->rc.ch[CATCH_Y_CHANNEL] / CATCH_OPEN_RC_SCALE;
    *vz_set = catch_RC->rc.ch[CATCH_Z_CHANNEL] / CATCH_OPEN_RC_SCALE;
}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Catch::solve()
{      
    for (int i = 0; i < 4; i++)
    {
        motor_set_control(&catch_motive_motor[i]);
    }

}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Catch::output()
{
    if (catch_behaviour_mode == CATCH_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            catch_motive_motor[i].current_give = 0.0f;
        }
    }
    can_receive.can_cmd_catch_motive_motor(catch_motive_motor[CAN_SPIN_L_MOTOR].current_give, catch_motive_motor[CAN_SPIN_R_MOTOR].current_give,
                                          catch_motive_motor[CAN_CATCH_YAW_MOTOR].current_give, catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].current_give);
    // can_receive.can_cmd_catch_motive_motor(0,0,0,0);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */

void Catch::motor_set_control(Mine_motor *motor)
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

void Catch::motor_angle_limit(Mine_motor *motor)
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


/**
 * @brief          自动模式控制电机转动角度
 * @param[out]     add: 角度增加量
 * @retval         none
 */
void Catch::auto_control(auto_mode_e *auto_mode)
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
            catch_motive_motor[CAN_SPIN_L_MOTOR].angle_set = + ROTATE_ANGLE[AUTO_MODE][SPIN_MOTOR];
            catch_motive_motor[CAN_SPIN_R_MOTOR].angle_set = - ROTATE_ANGLE[AUTO_MODE][SPIN_MOTOR];
            catch_motive_motor[CAN_CATCH_YAW_MOTOR].angle_set = - ROTATE_ANGLE[AUTO_MODE][YAW_MOTOR];
            catch_motive_motor[CAN_CATCH_SUCTION_MOTOR].angle_set = + ROTATE_ANGLE[AUTO_MODE][SUCTION_MOTOR];
        }
    }
}