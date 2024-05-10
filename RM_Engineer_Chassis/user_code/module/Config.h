#ifndef CONFIG_H
#define CONFIG_H

/*----------------------底盘---------------------------*/
//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0


/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 0


/*---------------------按键---------------------------*/

//底盘前后左右控制按键
#define KEY_PRESSED_CHASSIS_FRONT  'W'
#define KEY_PRESSED_CHASSIS_BACK   'S'
#define KEY_PRESSED_CHASSIS_LEFT   'A'
#define KEY_PRESSED_CHASSIS_RIGHT  'D'
#define KEY_PRESSED_UI_UPDATE      'B'

//抬升救援控制按键
#define KEY_PRESSED_CHASSIS_LIFT  'F'

#define KEY_PRESSED_INIT_STATE 'Z'

#define KEY_PRESSED_STANDARD_STATE 'X'

#define KEY_PRESSED_SKY_STATE 'C'

#define KEY_PRESSED_SUCKER_STATE 'F'

#endif
