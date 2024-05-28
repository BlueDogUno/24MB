#ifndef CONFIG_H
#define CONFIG_H

#define ANGLE_ERR_TOLERANT 5000

/*----------------------底盘---------------------------*/
//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0


/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯  底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 0

/*---------------------按键---------------------------*/



//初始模式 单击G
#define KEY_PRESSED_INIT_STATE     'Z'
//资源岛
#define KEY_PRESSED_STANDARD_STATE     'V'
//一键空接模式
#define KEY_PRESSED_SKY_STATE      'C'

//地矿
#define KEY_PRESSED_GROUND_STATE     'G'

//兑换
#define KEY_PRESSED_DELIVERY_STATE     'G'

//初始化UI界面
#define KEY_PRESSED_UI_UPDATE      'B'





#endif