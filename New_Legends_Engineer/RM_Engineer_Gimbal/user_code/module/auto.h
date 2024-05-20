#ifndef AUTO_H
#define AUTO_H

#include "system_config.h"

#include "struct_typedef.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"


// 自动模式各行为伸爪机构的相对长度
#define STRETCH_INIT_LEN 0.0f
#define STRETCH_SKY_LEN 400.0f
#define STRETCH_STANDARD_LEN 400.0f
#define STRETCH_GROUND_LEN 0.0f
#define STRETCH_DELIVERY_LEN 0.0f

// 自动模式各行为抓取机构的相对角度
#define ARM_INIT_SPIN_ANGLE 0.0f
#define ARM_INIT_YAW_ANGLE 0.0f
#define ARM_INIT_SUCTION_ANGLE 0.0f

#define ARM_STANDARD_SPIN_ANGLE 0.0f
#define ARM_STANDARD_YAW_ANGLE 0.0f
#define ARM_STANDARD_SUCTION_ANGLE 0.0f

#define ARM_GROUND_SPIN_ANGLE 0.0f
#define ARM_GROUND_YAW_ANGLE 0.0f
#define ARM_GROUND_SUCTION_ANGLE 0.0f

#define ARM_DELIVERY_SPIN_ANGLE 0.0f
#define ARM_DELIVERY_YAW_ANGLE 0.0f
#define ARM_DELIVERY_SUCTION_ANGLE 0.0f

typedef enum
{
    ARM_INIT,      //复位

    ARM_STANDARD,  //资源岛

    ARM_GROUND,    //地矿

    ARM_DELIVERY,  //兑矿

    AUTO_MODE_NUM,

} auto_mode_e;      //自动控制模式

typedef enum
{
    MINE_PUSH,

    MINE_STRETCH,

    MINE_SPIN,

    MINE_YAW,

    MINE_SUCTION,

    LIFT,   //抬升，需要从底盘发送数据上来

    PART_NUM,

} control_part;     //各控制机构

typedef enum
{
    WAIT = 0,

    READY,      

}motor_position_state;                    //电机是否到位

class AUTO
{
public:
    auto_mode_e auto_mode; //应该知道这是干嘛的吧
    auto_mode_e last_auto_mode;

    uint8_t auto_flag;

    uint8_t status_flag[PART_NUM];

    const RC_ctrl_t *auto_RC; //抓取机构使用的遥控器指针
    RC_ctrl_t *last_auto_RC; //抓取机构使用的遥控器指针

    void init();
    // 保证电机到达指定位置
    void motor_status_measure();
    void auto_control_set();
};

extern AUTO Auto;


#endif
