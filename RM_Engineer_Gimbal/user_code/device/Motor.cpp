#include "Motor.h"
#include "Pid.h"
#include "Can_receive.h"

void Joint_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}
