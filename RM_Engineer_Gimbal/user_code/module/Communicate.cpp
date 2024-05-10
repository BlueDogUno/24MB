#include "communicate.h"
#include "main.h"
#include "string.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h"

#ifdef __cplusplus
}
#endif

#include "detect_task.h"

#include "Remote_control.h"
#include "Can_receive.h"
#include "Referee.h"
#include "Ui.h"
Remote_control remote_control;
extern Can_receive can_receive;
Referee referee;
Ui ui;

Communicate communicate;

void Communicate::init()
{

// TODO 这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    ;
#else
    remote_control.init();
#endif

    can_receive.init();

    // referee.init();

    // ui.init(&referee.Judge_Self_ID, &referee.Judge_SelfClient_ID);
}

void Communicate::run()
{
    // referee.unpack();
    // referee.determine_ID();

    // ui.run();
    int16_t temp_ch0, temp_ch2, temp_ch3,temp_ch1;
    uint16_t temp_v;
    int8_t s0,s1,auto_mode;
    bool_t stretch_state,yaw_state,roll_state,flip_state;
    bool_t temp_sucker;


    temp_ch0 = remote_control.rc_ctrl.rc.ch[0];
    temp_ch1 = remote_control.rc_ctrl.rc.ch[1];
    temp_ch2 = remote_control.rc_ctrl.rc.ch[2];
    temp_ch3 = remote_control.rc_ctrl.rc.ch[3];
    temp_v = remote_control.rc_ctrl.key.v;
    s0 = remote_control.rc_ctrl.rc.s[0];
    s1 = remote_control.rc_ctrl.rc.s[1];





    can_receive.send_rc_board_com(temp_ch0, temp_ch2, temp_ch3, temp_v);
    can_receive.send_high_state_com( stretch_state ,  yaw_state,  roll_state,  flip_state);
    can_receive.send_ss_state_com( s0,s1,temp_ch1,temp_sucker);
    can_receive.send_lift_auto_mode( auto_mode);

}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
    // TODO 设备检测未更新
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (hcan == &ARM_COM) 
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
                case CAN_ROLL_ID: 
				case CAN_Y_SLID_ID:
				case CAN_X_STRETCH_ID:           
				{
					static int j;
					j = rx_header.StdId - CAN_ROLL_ID;
					can_receive.get_arm_motor_measure(j,rx_data);
					break;
				}
                default:
                {
                    break;
                }
            }
        }
        else if (hcan == &DM_ARM_COM) //接底盘CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
                case CAN_DAMIAO_YAW:
                case CAN_DAMIAO_PITCH:
                {
                    //达妙接收
                }
                default:
                {
                    break;
                }
            }
        }
        
    }
    // TODO 设备检查未更新
    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(0);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(1);
                }
            }
        }
    }

    // TODO 设备检查未更新
    //裁判串口数据
    void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[0]), this_time_rx_len);
                // detect_hook(REFEREE_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[1]), this_time_rx_len);
                // detect_hook(REFEREE_TOE);
            }
        }
    }
}
#endif