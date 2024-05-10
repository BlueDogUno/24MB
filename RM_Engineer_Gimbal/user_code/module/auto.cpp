/***      
 *    《无敌的自动模式》
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

#include "auto.h"
#include "Communicate.h"
#include "arm_ctrl.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

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

AUTO Auto;
extern Arm_ctrl arm_ctrl;

void AUTO::init()
{
    // auto_mode = ARM_INIT;
    // last_auto_mode = auto_mode;
    // arm_ctrl.auto_control(&auto_mode);
    // auto_RC = remote_control.get_remote_control_point();
    // last_auto_RC = remote_control.get_last_remote_control_point();
}

/**
 * @brief          保证电机到达指定位置后设置标志位
 * @param[out]

 * @retval         none
 */
void AUTO::motor_status_measure()
{
    // // 抓取机构
    // if (arm_ctrl.motor_status[CAN_SPIN_L_MOTOR] == READY && arm_ctrl.motor_status[CAN_SPIN_R_MOTOR] == READY)
    //     status_flag[MINE_SPIN] = READY;
    // else
    //     status_flag[MINE_SPIN] = WAIT;
    
    // if (arm_ctrl.motor_status[CAN_ARM_YAW_MOTOR] == READY)
    //     status_flag[MINE_YAW] = READY;
    // else
    //     status_flag[MINE_YAW] = WAIT;
    
    // if (arm_ctrl.motor_status[CAN_ARM_SUCTION_MOTOR] == READY)
    //     status_flag[MINE_SUCTION] = READY;
    // else
    //     status_flag[MINE_SUCTION] = WAIT;
    
    // // 伸爪机构
    // if (arm_ctrl.motor_status[MINE_STRETCH_L_ID] == READY && arm_ctrl.motor_status[MINE_STRETCH_R_ID] == READY)
    //     status_flag[MINE_STRETCH] = READY;
    // else
    //     status_flag[MINE_STRETCH] = WAIT;


}

/**
 * @brief          自动控制
 * @param[out]
 * @retval         none
 */
void AUTO::auto_control_set()
{
    // if (if_key_pessed(auto_RC, KEY_PRESSED_INIT_STATE))
    // {
    //     auto_mode = ARM_INIT;
    //     // if (status_flag[MINE_SPIN] != READY || status_flag[MINE_YAW] != READY || status_flag[MINE_SUCTION] != READY)
    //     arm_ctrl.auto_control(&auto_mode);
    //     // else if (status_flag[MINE_STRETCH] != READY)
    //     // minepush.auto_control(&auto_mode);
    // }
    // else if (if_key_pessed(auto_RC, KEY_PRESSED_STANDARD_STATE))
    // {
    //     auto_mode = ARM_STANDARD;
    //     // if (status_flag[MINE_SPIN] != READY || status_flag[MINE_YAW] != READY || status_flag[MINE_SUCTION] != READY)
    //     arm_ctrl.auto_control(&auto_mode);
    //     // else if (status_flag[MINE_STRETCH] != READY)
    //     // minepush.auto_control(&auto_mode);
    // }

}
