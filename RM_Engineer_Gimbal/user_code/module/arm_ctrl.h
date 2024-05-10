#ifndef ARM_CTRL_H
#define ARM_CTRL_H

#include "system_config.h"
#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "auto.h"
#include "DMPower.h"

#define ARM_DEBUG_FLAG 0
#define TERMINAL_POSTURE_MOTOR_NUM 3 //末端大疆电机数量和拨矿电机
#define WRIST_DM_MOTOR_NUM 3 //腕部达妙电机数量
#define ELBOW_MOTOR_NUM 2 //肘部大疆电机数量


//任务控制间隔 2ms
#define ARM_CONTROL_TIME_MS 2

#define ARM_OPEN_RC_SCALE 300 // rz遥控器除以该比例发送到can上

//m3508转化成速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define ARM_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define ARM_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//-------------------------------遥控器通道定义----------------------------------------
//控制变换的遥控器摇杆通道 左摇杆上下
#define ARM_CTRL_CHANNEL 3 

//左拨杆 模式切换拨杆
#define MODE_SWITCH_CHANNEL 0
//终端姿态模式遥控器左拨杆  左上
//肘部位置模式遥控器左拨杆  左中

//右拨杆 关节选择拨杆
#define JOINT_CHOOSE_CHANNEL 1
//------------------------------限幅参数定义--------------------------------------------

//pitch轴角度限幅
#define SPIN_LIMIT_ANGLE 100.0f 
//yaw轴角度限幅
#define YAW_LIMIT_ANGLE 100.0f  
//roll轴角度限幅
#define SUCTION_LIMIT_ANGLE 100.0f 
//x_stretch轴角度限幅
#define X_STRENTC_LIMIT_ANGLE 0.0f
//y_slid轴角度限幅
#define Y_SLID_LIMIT_ANGLE 0.0f

//最大速度
#define NORMAL_MAX_ARM_SPEED 6.0f //2.0
//出爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 2.0f //2.0




//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0

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



//用于电机控制编号，不一定是can通讯id
typedef enum{
    
    CAN_TERMINAL_ROLL ,
    CAN_TERMINAL_PITCH ,
    
    CAN_WRIST_DM_ROLL ,
    CAN_WRIST_DM_PITCH ,
    CAN_WRIST_DM_YAW ,
    
    CAN_ELBOW_X_STRETCH ,
    CAN_ELBOW_Y_SLID ,
    
    CAN_POSTURE,

}arm_motor_id;

typedef enum
{
    ARM_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    ARM_CLOSE,                       //全自动，操作手没有权限控制                     

    ARM_WRIST_YAW,                   //腕部yaw轴电机

    ARM_WRIST_PITCH,                 //腕部pitch轴电机

    ARM_WRIST_ROLL,                  //腕部roll轴电机

    ARM_TERMINAL_ROLL,               //末端roll轴

    ARM_TERMINAL_PITCH,              //末端pitch轴

    ARM_X_STRETCH,                   //x_出爪

    ARM_Y_SLID,                      //y_横移

    ARM_HOLD,

} arm_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    ARM_AUTO,

    ARM_HAND,

    ARM_HOLD_MODE,

} arm_mode_e;      //控制模式 大类？



class Arm_ctrl
{
public:
    const RC_ctrl_t *arm_RC; //臂机构使用的遥控器指针
    RC_ctrl_t *last_arm_RC; //臂机构使用的遥控器指针
    uint16_t arm_last_key_v;  //遥控器上次按键

    arm_behaviour_e arm_behaviour_mode; //臂机构行为状态机
    arm_behaviour_e last_arm_behaviour_mode; //臂机构上次行为状态机

    arm_mode_e arm_mode; //臂机构控制状态机
    arm_mode_e last_arm_mode; //臂机构上次控制状态机

    //定义终端姿态电机
    Joint_motor terminal_motor[TERMINAL_POSTURE_MOTOR_NUM]; 
    DM_Motor_t terminal_DM_motor[TERMINAL_DM_MOTOR_NUM];
    int32_t terminal_motor_start_angle[TERMINAL_POSTURE_MOTOR_NUM];
    //定义肘部位置电机
    Joint_motor elbow_motor[ELBOW_MOTOR_NUM];
    int32_t elbow_motor_start_angle[ELBOW_MOTOR_NUM];

    //电机状态
    uint8_t terminal_motor_status[TERMINAL_DM_MOTOR_NUM];
    uint8_t elbow_motor_status[ELBOW_MOTOR_NUM];

    //终端电机维持角度
    fp32 terminal_hold_angle[TERMINAL_POSTURE_MOTOR_NUM];
    //肘部电机维持角度
    fp32 elbow_hold_angle[ELBOW_MOTOR_NUM];

    
    //主流程
    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();

    void auto_control(auto_mode_e *auto_mode);

    void solve();

    void output();

    //行为模式
    void behaviour_control_set(fp32 *vyaw_set, fp32 *vpitch_set,fp32 *vroll_set,fp32 *vx_set,fp32 *vy_set);

    void arm_open_set_control(fp32 *vyaw_set, fp32 *vpitch_set,fp32 *vroll_set,fp32 *vx_set,fp32 *vy_set);

    void motor_set_control(Joint_motor *motor);

    //功能函数
    void motor_angle_limit(Joint_motor *motor);
        
};

extern Arm_ctrl arm_ctrl;

//------------------------------------------pid参数 TODO不是实际数值需调整----------------------------------------
//roll轴电机速度环
#define ROLL_MOTOR_SPEED_PID_KP  10.0f
#define ROLL_MOTOR_SPEED_PID_KI  0.0f
#define ROLL_MOTOR_SPEED_PID_KD  0.0f
#define ROLL_MOTOR_SPEED_PID_MAX_IOUT 1.0f
#define ROLL_MOTOR_SPEED_PID_MAX_OUT 2000.0f

//roll轴电机角度环  
#define ROLL_MOTOR_ANGLE_PID_KP 3.0f 
#define ROLL_MOTOR_ANGLE_PID_KI 0.0f
#define ROLL_MOTOR_ANGLE_PID_KD 0.0f
#define ROLL_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define ROLL_MOTOR_ANGLE_PID_MAX_OUT 11000.0f

//yaw轴电机速度环PID
#define YAW_MOTOR_SPEED_PID_KP 1000.0f
#define YAW_MOTOR_SPEED_PID_KI 0.0f
#define YAW_MOTOR_SPEED_PID_KD 2000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 10000.0f

//yaw轴电机角度环PID
#define YAW_MOTOR_ANGLE_PID_KP 3.0f 
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 11000.0f

// pitch轴 电机速度环PID        
#define PITCH_MOTOR_SPEED_PID_KP 1000.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.0f
#define PITCH_MOTOR_SPEED_PID_KD 2000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 10000.0f

// pitch轴 电机角度环PID
#define PITCH_MOTOR_ANGLE_PID_KP 3.0f 
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 11000.0f

//x轴出爪电机速度环PID   无需角度环
#define X_STRETCH_MOTOR_SPEED_PID_KP 100.0f
#define X_STRETCH_MOTOR_SPEED_PID_KI 200.0f
#define X_STRETCH_MOTOR_SPEED_PID_KD 0.0f
#define X_STRETCH_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define X_STRETCH_MOTOR_SPEED_PID_MAX_OUT 5000.0f

//y轴横移电机速度环
#define Y_SLID_MOTOR_SPEED_PID_KP 100.0f
#define Y_SLID_MOTOR_SPEED_PID_KI 200.0f
#define Y_SLID_MOTOR_SPEED_PID_KD 0.0f
#define Y_SLID_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define Y_SLID_MOTOR_SPEED_PID_MAX_OUT 5000.0f

#endif
