/*
 * MOTOR_PID_CONTROL.h
 *
 *  Created on: Nov 20, 2023
 *      Author: minht
 */

#ifndef MOTOR_PID_CONTROL_H_
#define MOTOR_PID_CONTROL_H_

#include "MOTOR_DRIVER.h"
#include "PID.h"
#include "math.h"
typedef struct
{
    PID_TypeDef PIDVelocity;
    PID_TypeDef PIDPosition;
    MOTOR_DRIVER_t *motorDriver;
    uint16_t pinSetHome;
    float setPoint;
    float preSetPoint;
    float error;
    float preError;
    float ratioJoint;
    float pos;
    float speed;
}MOTOR_t;
void MOTOR_setPIDVelocity(MOTOR_t* motor, float Kp, float Ki, float Kd, float Ts);
void MOTOR_setPIDPosition(MOTOR_t* motor, float Kp, float Ki, float Kd, float Ts);
void MOTOR_setAngle(MOTOR_t* motor, float setAngle);
void MOTOR_setWindupRange(MOTOR_t* motor, float OutMin, float OutMax);
void MOTOR_setOutputRange(MOTOR_t* motor, float OutMin, float OutMax);
void MOTOR_runAngle(MOTOR_t* motor);
void MOTOR_init(MOTOR_t* motor, MOTOR_DRIVER_t* motorDriver, float ratio, uint16_t pinSetHome);

#endif /* MOTOR_PID_CONTROL_H_ */
