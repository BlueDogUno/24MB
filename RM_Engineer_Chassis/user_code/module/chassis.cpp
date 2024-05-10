#include "Chassis.h"
#include "Communicate.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "Ui.h"
#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif
//ui模块
extern Ui ui;
//底盘模块 对象
Chassis chassis;

/**
 * @brief          初始化变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]
 * @retval         none
 */
void Chassis::init()
{
    //获取遥控器指针
    chassis_RC = remote_control.get_remote_control_point();
    last_chassis_RC = remote_control.get_last_remote_control_point();

    chassis_last_key_v = 0;

    //设置初试状态机
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    chassis_mode = CHASSIS_VECTOR_RAW;
    last_chassis_mode = chassis_mode;

    //初始化底盘电机
    for (uint8_t i = 0; i < 4; ++i)
    {

        //动力电机数据
        chassis_motive_motor[i].init(can_receive.get_chassis_motive_motor_measure_point(i));
        //初始化pid
        fp32 motive_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        chassis_motive_motor[i].speed_pid.init(PID_SPEED, motive_speed_pid_parm, &chassis_motive_motor[i].speed, &chassis_motive_motor[i].speed_set, NULL);
        chassis_motive_motor[i].speed_pid.pid_clear();
    }

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //初始化角度Z轴PID
    fp32 z_angle_pid_parm[5] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT};
    chassis_wz_angle_pid.init(PID_ANGLE, z_angle_pid_parm, &chassis_relative_angle, &chassis_relative_angle_set, NULL);
    chassis_wz_angle_pid.pid_clear();
    //速度限幅设置
    x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;

    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;

    z.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    z.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    //更新一下数据
    feedback_update();
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]
 * @retval         none
 */
void Chassis::set_mode()
{
    chassis_behaviour_mode_set();
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]
 * @retval         none
 */
void Chassis::feedback_update()
{
    //记录上一次遥控器值
    // last_chassis_RC->key.v = chassis_RC->key.v;
    chassis_last_key_v = chassis_RC->key.v;

    //切入跟随云台模式
    if ((last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW))
    {
        chassis_relative_angle_set = INIT_YAW_SET;
    }
    //切入不跟随云台模式
    else if ((last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW))
    {
        chassis_yaw_set = 0;
    }
    //切入不跟随云台模式
    else if ((last_chassis_mode != CHASSIS_VECTOR_RAW) && (chassis_mode == CHASSIS_VECTOR_RAW))
    {
        chassis_yaw_set = 0;
    }

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度，加速度是速度的PID微分
        chassis_motive_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_motive_motor[i].motor_measure->speed_rpm;
        chassis_motive_motor[i].accel = chassis_motive_motor[i].speed_pid.data.error_delta * CHASSIS_CONTROL_FREQUENCE;
    }

    //更新底盘x, y, z速度值,右手坐标系
    // TODO 速度的更新可能要进行修改
    x.speed = (-chassis_motive_motor[0].speed + chassis_motive_motor[1].speed + chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    y.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed + chassis_motive_motor[2].speed + chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    z.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed - chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}

fp32 move_top_xyz_parm[3] = {1.0, 1.0, 1.3};

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Chassis::set_contorl()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

    //获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);

    if (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //“angle_set” 是旋转速度控制
        z.speed_set = angle_set;
        //速度限幅
        x.speed_set = fp32_constrain(vx_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
    }
    else if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //在原始模式，设置值是发送到CAN总线
        x.speed_set = vx_set;
        y.speed_set = vy_set;
        z.speed_set = angle_set;
        chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Chassis::solve()
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //动力电机目标速度

    uint8_t i = 0;

    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(wheel_speed);

    if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].current_give = (int16_t)(wheel_speed[i]);
        }
        // raw控制直接返回
        return;
    }

    //计算动力电机控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_motive_motor[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].speed_set *= vector_rate;
        }
    }

    //计算pid
    for (i = 0; i < 4; i++)
    {
        //计算动力电机的输出电流
        chassis_motive_motor[i].current_set = chassis_motive_motor[i].speed_pid.pid_calc();
    }
}

fp32 chassis_power = 0.0f;
fp32 chassis_power_limit = 0.0f;


/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Chassis::output()
{
    //赋值电流值
    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].current_give = (int16_t)(chassis_motive_motor[i].current_set);
        if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
        {
            chassis_motive_motor[i].current_give = 0;
        }
    }

    //电流输出控制,通过调整宏定义控制
    for (int i = 0; i < 4; i++)
    {
#if CHASSIS_MOTIVE_MOTOR_NO_CURRENT
        chassis_motive_motor[i].current_give = 0;
#endif
    }

    can_receive.can_cmd_chassis_motive_motor(chassis_motive_motor[0].current_give, chassis_motive_motor[1].current_give,
                                             chassis_motive_motor[2].current_give, chassis_motive_motor[3].current_give);
}

/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Chassis::chassis_behaviour_mode_set()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    last_chassis_mode = chassis_mode;

    // //遥控器设置模式
    // if (switch_is_up(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))//右拨杆上  底盘无力minepush
    // {
    //     chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    // }
    // else if (switch_is_mid(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //右拨杆中  
    // {
    //     chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    // }
    // else if (switch_is_down(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //右拨杆下  底盘自主运动
    // {
    //     chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    // }
    // if (switch_is_mid(chassis_RC->rc.s[0])) //左拨杆下  底盘无力状态
    // {
    //     chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    // }
    if (switch_is_down(chassis_RC->rc.s[LEFT_CHANNEL]) && switch_is_down(chassis_RC->rc.s[RIGHT_CHANNEL])) //左拨杆下 右拨杆下 底盘无力
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }else                                                       //其他情况下开启底盘 
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }

    //添加自己的逻辑判断进入新模式

    //根据行为模式选择一个底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) //底盘控制 开环 直接将遥控器杆量转化为电流值 当前逻辑表现为无力
    {
        chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) //底盘控制 闭环 跟随云台
    {
        chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) //底盘控制 闭环 自主运动
    {
        chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]       包括底盘所有信息.
 * @retval         none
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW )
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN) //底盘控制 测试用 直接将遥控器杆量转化为电流值
    {
        chassis_open_set_control(vx_set, vy_set, angle_set);
    }
    last_chassis_RC->key.v = chassis_RC->key.v;
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @retval         返回空
 */
void Chassis::chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @retval         返回空
 */
void Chassis::chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @retval         返回空
 */
void Chassis::chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set);

	/****************************重新绘制UI*********************************************/	
    if(KEY_UI_UPDATE){
	     ui.start();
    }

}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         返回空
 */
void Chassis::chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set);
       
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]-chassis_RC->mouse.x * CHASSIS_WZ_RC_SEN;
    
    

}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void Chassis::chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    *vx_set = chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @retval         none
 */
void Chassis::chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    

    //键盘控制
    if (KEY_CHASSIS_FRONT)
    {
        vx_set_channel = 0.5f;
    }
    else if (KEY_CHASSIS_BACK)
    {
        vx_set_channel = -0.5f;
    }

    if (KEY_CHASSIS_LEFT)
    {
        vy_set_channel = 0.5f;
    }
    else if (KEY_CHASSIS_RIGHT)
    {
        vy_set_channel = -0.5f;
    }

    // //一阶低通滤波代替斜波作为底盘速度输入
    // chassis_cmd_slow_set_vx.first_order_filter_cali(vx_set_channel);
    // chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);

    // //停止信号，不需要缓慢加速，直接减速到零
    // if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    // {
    //     chassis_cmd_slow_set_vx.out = 0.0f;
    // }

    // if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    // {
    //     chassis_cmd_slow_set_vy.out = 0.0f;
    // }

    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;

}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
void Chassis::chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4])
{

    // normal
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    //  wheel_speed[0] = -x.speed_set - y.speed_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    //  wheel_speed[1] = x.speed_set - y.speed_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    //  wheel_speed[2] = x.speed_set + y.speed_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    //  wheel_speed[3] = -x.speed_set + y.speed_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * z.speed_set;

    wheel_speed[0] = -x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[1] = x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[2] = x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[3] = -x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
fp32 Chassis::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


