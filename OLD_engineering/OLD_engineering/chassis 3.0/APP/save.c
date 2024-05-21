#include "save.h"
#include "tim.h"
#include "remote_control.h"



static void save_init(void)
{
	save.rc_data = get_remote_control_point();
}

void gimbal_init(void)
{
	save.gimbal_angle = 1700;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, save.gimbal_angle);
}

int8_t ore_data = 0;
int8_t ore_last_data = 0;
int8_t ore_data_1 = 0;

int8_t save_data =0;

int8_t save_key_board = 1;
void save_task(void const * argument)
{
	save_init();
	gimbal_init();
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
	while (1)
	{
		if(LEFT_SWITCH_DOWN&&RIGHT_SWITCH_UP)
		{
			save_key_board = 1;
		}else{
			save_key_board = 0;
		}
		//地矿
		ore_last_data = ore_data_1;
		if(save.rc_data->key.v == KEY_PRESSED_OFFSET_G)
		{
			ore_data_1 = 1;
		}else
		{
			ore_data_1 = 0;
		}
		if(ore_data_1 != ore_last_data && ore_data_1)
		{
			if(ore_data == 1)
			{
				ore_data = 0;
			}else
			{
				ore_data = 1;
			}
		}
		// if (ore_data == 0)
		// {
		// 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		// }else
		// {
		// 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		// }
		//救援
		if(save_key_board == 0)
		{
			// if (LEFT_SWITCH_DOWN && RIGHT_SWITCH_MID)
			// {
			// 	if (LEFT_CH_DOWN)
			// 	{
			// 		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 970);
			// 		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2030);
			// 		//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);
			// 	}
			// 	else if(LEFT_CH_UP)
			// 	{
			// 		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
			// 		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
			// 		//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_SET);
			// 	}
			// }
		}else
		{
			if(save.rc_data->mouse.z > 0&&save.rc_data->key.v == KEY_PRESSED_OFFSET_F)
			{
				save_data = 1;
			}
			if(save.rc_data->mouse.z < 0&&save.rc_data->key.v == KEY_PRESSED_OFFSET_F)
			{
				save_data = 0;
			}
			if (save_data == 0)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
			}else
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1030);//1450
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1980);//2300
			}
		}
		
		//云台
		if(save.rc_data->mouse.press_l == 0 && save.rc_data->mouse.press_r == 0)
		{
			save.gimbal_angle = 1700;
		}
		if(save.rc_data->mouse.press_l == 1 && save.rc_data->mouse.press_r == 0)
		{
			save.gimbal_angle = 1367;
		}
		if(save.rc_data->mouse.press_r == 1 && save.rc_data->mouse.press_l == 0)
		{
			save.gimbal_angle = 2033;
		}
		if(save.rc_data->mouse.press_r == 1 && save.rc_data->mouse.press_l == 1)
		{
			save.gimbal_angle = 900;
		}
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, save.gimbal_angle);
	}
}

