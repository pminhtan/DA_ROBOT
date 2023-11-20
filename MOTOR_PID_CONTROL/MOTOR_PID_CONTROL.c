/*
 * MOTOR_PID_CONTROL.c
 *
 *  Created on: Nov 20, 2023
 *      Author: minht
 */
#include "MOTOR_PID_CONTROL.h"
void MOTOR_setPIDVelocity(MOTOR_t* motor, float Kp, float Ki, float Kd, float Ts)
{
    PID_init(&motor->PIDVelocity, Kp, Ki, Kd, Ts);
}
void MOTOR_setPIDPosition(MOTOR_t* motor, float Kp, float Ki, float Kd, float Ts)
{
    PID_init(&motor->PIDPosition, Kp, Ki, Kd, Ts);
}
void MOTOR_setOutputRange(MOTOR_t* motor, float OutMin, float OutMax)
{
    PID_setOutputRange(&motor->PIDPosition, -200, 200);
    PID_setOutputRange(&motor->PIDVelocity, OutMin, OutMax);
}
void MOTOR_setWindupRange(MOTOR_t* motor, float OutMin, float OutMax)
{
    PID_setWindupGain(&motor->PIDVelocity, 0);
    PID_setWindupRange(&motor->PIDVelocity, OutMin, OutMax);
}
void MOTOR_setAngle(MOTOR_t* motor, float setAngle)
{
    motor->setPoint = setAngle * motor->ratioJoint;
}
void MOTOR_runAngle(MOTOR_t* motor)
{
    if (motor->setPoint != motor->preSetPoint)
    {
        PID_clear(&motor->PIDPosition);
        PID_clear(&motor->PIDVelocity);
        motor->preSetPoint = motor->setPoint;
    }
    MOTOR_driver_readPosAndSpeed(motor->motorDriver, &motor->pos, &motor->speed, motor->PIDPosition.Sample_time);
    motor->error = motor->setPoint - motor->pos;

    // PD+PI controller
    PID_compute(&motor->PIDPosition, motor->error);  // tinh PD
    PID_compute(&motor->PIDVelocity, motor->PIDPosition.Output - motor->speed); // tinh PI

    if (fabs(motor->pos) < 1500)
    {
        MOTOR_driver_rotary(motor->motorDriver, motor->PIDVelocity.Output);
    }
    else
    {
        MOTOR_driver_rotary(motor->motorDriver, 0);
    }
}
void MOTOR_init(MOTOR_t* motor, MOTOR_DRIVER_t *motorDriver,float ratio, uint16_t pinSetHome)
{
    motor->ratioJoint = ratio;
    motor->pinSetHome = 0;
    motor->setPoint = 0;
    motor->preSetPoint = 0;
    motor->error = 0;
    motor->preError = 0;
    motor->pos = 0;
    motor->speed = 0;
    motor->motorDriver = motorDriver;
}
void MOTOR_reset(MOTOR_t* motor)
{
    motor->pinSetHome = 0;
    motor->setPoint = 0;
    motor->preSetPoint = 0;
    motor->error = 0;
    motor->preError = 0;
    motor->pos = 0;
    motor->speed = 0;
    PID_clear(&motor->PIDPosition);
    PID_clear(&motor->PIDVelocity);
}
