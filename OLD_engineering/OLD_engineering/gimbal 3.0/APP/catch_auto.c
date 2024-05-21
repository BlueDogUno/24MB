/*
    自动模式，包含自动夹矿，自动收纳，自动放置，自动推入，自动归位五个功能
    自动模式对于操作手而言非常重要，工程一定要有这个功能！！！！！！！！！
    如果让操作手手动夹矿，小组赛都不可能出线，太慢了
*/

#include "catch_auto.h"
#include "gpio.h"

auto_t auto_ctrl;
int16_t delay_i;

int16_t close_to(void)
{
    int16_t lift_judge = 0;
    int16_t flip_judge = 0;
    int16_t stretch_judge = 0;
    int16_t catch_judge = 0;

    if(auto_ctrl.a_flip_target - (int16_t)auto_ctrl.a_flip_angle < 7 && (int16_t)auto_ctrl.a_flip_angle - auto_ctrl.a_flip_target < 7)
    {
        flip_judge = 1;
    }
    if(auto_ctrl.a_lift_target - auto_ctrl.a_lift_angle < 5.0f && auto_ctrl.a_lift_angle - auto_ctrl.a_lift_target < 5.0f)
    {
        lift_judge = 1;
    }
    if(auto_ctrl.a_stretch_target - auto_ctrl.a_stretch_angle < 5.0f && auto_ctrl.a_stretch_angle - auto_ctrl.a_stretch_target < 5.0f)
    {
        stretch_judge = 1;
    }
    if(auto_ctrl.motor[3]->speed_rpm < 30)
    {
        catch_judge = 1;
    }

    if(lift_judge && flip_judge && stretch_judge)
    {
        return 1;
    }else{
        return 0;
    }
}       

void catch_auto_init(void)
{
    auto_ctrl.rc_data = get_remote_control_point();

    auto_ctrl.press_flag = 1;
    auto_ctrl.auto_mode = 0;

    auto_ctrl.a_catch_mode = 0;
    auto_ctrl.a_takein_mode = 0;
    auto_ctrl.a_takeout_mode = 0;
    auto_ctrl.a_exchange_mode = 0;

    auto_ctrl.a_lift_target = 0;
    auto_ctrl.a_stretch_target = 0;
    // auto_ctrl.a_flip_target = 0.0f;
    auto_ctrl.a_catch_target = 0;

    auto_ctrl.flip_reset_angle = 0.0;

    for (int i = 0; i < 8; i++)
    {
        auto_ctrl.motor[i] = get_motor_measure_point(i);
    }

}
// 空接广电，但后来发现这样子做的话空接太慢，大约慢0.2秒，对于单片机来说已经是非常慢了
// 后来打比赛的时候就用来自动夹取，操作手只要打开自动夹矿，对准之后怼上去就能夹到
// void sensor(void)
// {
//     if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
//     {
//         auto_ctrl.photogate_1  =   0;
//     }
//     if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
//     {
//         auto_ctrl.photogate_1  =   1;
//     }
// }

void measure(void)
{   
    auto_ctrl.flip_reset_flag = auto_ctrl.flip_reset_last_flag;
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        auto_ctrl.flip_reset_flag = 1;        
    }else{
        auto_ctrl.flip_reset_flag = 0;   
    }
    if(auto_ctrl.flip_reset_flag != auto_ctrl.flip_reset_last_flag)
    {
        auto_ctrl.flip_reset_angle = auto_ctrl.a_flip_angle;
    }
    
    auto_ctrl.a_flip_angle = 1.0*(auto_ctrl.motor[0]->round*360)/19+1.0*(auto_ctrl.motor[0]->ecd*360)/19/8192;
    auto_ctrl.a_stretch_angle = 1.0*(auto_ctrl.motor[2]->round*360)/19+1.0*(auto_ctrl.motor[2]->ecd*360)/19/8192;
    auto_ctrl.a_lift_angle = 1.0*(auto_ctrl.motor[4]->round*360)/19+1.0*(auto_ctrl.motor[4]->ecd*360)/19/8192;
}

void auto_set_mode(void)
{
    auto_ctrl.last_press_flag = auto_ctrl.press_flag;
    if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_R)
    {
        auto_ctrl.press_flag = 1;
    }else{
        auto_ctrl.press_flag = 0;
    }

    if(auto_ctrl.last_press_flag != auto_ctrl.press_flag && auto_ctrl.press_flag == 1)
    {
        if(auto_ctrl.auto_mode == 1)
        {
            auto_ctrl.auto_mode = 0;

            auto_ctrl.a_catch_target = 1;
            auto_ctrl.a_catch_mode = 0;
            auto_ctrl.a_takein_mode = 0;
            auto_ctrl.a_takeout_mode = 0;
            auto_ctrl.a_push_mode = 0;
            auto_ctrl.a_exchange_mode = 0;
        }else{
            auto_ctrl.auto_mode = 1;
        }
    }

    if(auto_ctrl.auto_mode == 1)
    {
        // 自动取矿
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_Z)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0)
            {
                auto_ctrl.a_catch_mode = 1;
            }
        }
        // 自动收纳
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0)
            {
                auto_ctrl.a_takein_mode = 1;
                auto_ctrl.a_takein_flag = 1;
            }
        }
        // 自动放置
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0)
            {
                auto_ctrl.a_takeout_mode = 1;
            }
        }
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_V)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0)
            {
                auto_ctrl.a_push_mode = 1;
                auto_ctrl.a_push_flag = 0;
            }
        }
        // 自动归位
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0)
            {
                auto_ctrl.a_exchange_mode = 1;
            }
        }
    }else{
        auto_ctrl.a_catch_target = 1;//夹爪夹紧

        auto_ctrl.a_catch_mode = 0;//模式初始化
        auto_ctrl.a_takein_mode = 0;
        auto_ctrl.a_takeout_mode = 0;
        auto_ctrl.a_push_mode = 0;
        auto_ctrl.a_exchange_mode = 0;

        auto_ctrl.target_mode = 0;
    }
}

void auto_control(void)
{
    // 自动取矿
    if(auto_ctrl.a_catch_mode == 1)
    {
        auto_ctrl.target_mode = 1;

        auto_ctrl.a_lift_target = -120.0f;
        auto_ctrl.a_stretch_target = 1050.0f;
        auto_ctrl.a_flip_target = 90 + (int16_t)auto_ctrl.flip_reset_angle;
        auto_ctrl.a_catch_target = 0;
       
        auto_ctrl.arrive = close_to(); 
        if(auto_ctrl.arrive == 1)
        {
            auto_ctrl.a_catch_mode = 0;
        }
    }
    if(auto_ctrl.a_lift_target == -120.0f && auto_ctrl.a_stretch_target == 1050.0f && auto_ctrl.a_flip_target == 90 + (int16_t)auto_ctrl.flip_reset_angle && auto_ctrl.a_catch_target == 0)
    {
        if(auto_ctrl.photogate_1 == 0)
        {
            auto_ctrl.a_catch_target = 1;
        }else{
            auto_ctrl.a_catch_target = 0;
        }
    }
    // 自动收纳
    if(auto_ctrl.a_takein_mode == 1)
    {   
        auto_ctrl.target_mode = 1;

        auto_ctrl.a_catch_target = 1;

        if(auto_ctrl.motor[3]->given_current > 4000 && (auto_ctrl.motor[3]->speed_rpm < 5 && auto_ctrl.motor[3]->speed_rpm > -5))
        {
            auto_ctrl.a_takein_flag = 1;
        }
        if(auto_ctrl.a_takein_flag == 1)
        {
            auto_ctrl.a_catch_target = 1;
            auto_ctrl.a_takein_mode = 0;
            if(close_to())
            {
                auto_ctrl.a_catch_target = 1;
                auto_ctrl.a_stretch_target = 280.0f;
                auto_ctrl.a_flip_target = 245 + (int16_t)auto_ctrl.flip_reset_angle;
                auto_ctrl.a_takein_mode = 0;
            }
                // if(delay_i == 5000)
                // {
                //     delay_i = 0;
                //     auto_ctrl.a_stretch_target = 85.0f;
                //     if(280.0f - auto_ctrl.a_stretch_target < 1.0f && auto_ctrl.a_flip_target - 280.0f < 1.0f)
                //     {
                //         auto_ctrl.a_flip_target = 190;
                //         auto_ctrl.a_takein_mode = 0;
                //     }
                // }
        }
        // auto_ctrl.a_lift_target = 0;
        // auto_ctrl.a_stretch_target = 0;
        // auto_ctrl.a_flip_target = 0;
        // auto_ctrl.a_catch_target = 0;
    }
    // 自动放置
    if(auto_ctrl.a_takeout_mode == 1)
    {
        auto_ctrl.target_mode = 1;

        auto_ctrl.a_lift_target = -400.0f;
        auto_ctrl.a_flip_target = 110 + (int16_t)auto_ctrl.flip_reset_angle;
        auto_ctrl.a_catch_target = 1;

        if(close_to())
        {
            auto_ctrl.a_catch_target = 0;
            auto_ctrl.a_takeout_mode = 0;
        }
        // auto_ctrl.a_catch_target = 0;
    }
    // 自动推进
    if(auto_ctrl.a_push_mode == 1)
    {   
        auto_ctrl.target_mode = 1;

        if(auto_ctrl.a_push_flag == 0)
        {
            auto_ctrl.a_flip_target = 90 + (int16_t)auto_ctrl.flip_reset_angle;
            auto_ctrl.a_stretch_target = 0.0f;
            auto_ctrl.a_catch_target = 0;
        }
        if(auto_ctrl.a_push_flag == 1)
        {
            auto_ctrl.a_catch_target = 1;
            auto_ctrl.a_stretch_target = 1050.0f;
            auto_ctrl.a_push_mode = 0;
        }
        if(close_to())
        {
            auto_ctrl.a_push_flag = 1;
        }
    }
    // 自动归位
    if(auto_ctrl.a_exchange_mode == 1)
    {
        auto_ctrl.target_mode = 1;

        auto_ctrl.a_catch_target = 1;
        auto_ctrl.a_lift_target = 0.0;
        auto_ctrl.a_stretch_target = 0.0;
        auto_ctrl.a_flip_target = 0 + (int16_t)auto_ctrl.flip_reset_angle;
        if(close_to())
        {
            auto_ctrl.a_exchange_mode = 0;
        }
        // auto_ctrl.a_catch_target = 0;
        
    }
}

const auto_t *get_auto_control_point(void)
{
    return &auto_ctrl;
}

void StartDefaultTask(void const * argument)
{
    catch_auto_init();
    while (1)
    {
        sensor();
        measure();
        auto_control();
        auto_set_mode();
    }
}
