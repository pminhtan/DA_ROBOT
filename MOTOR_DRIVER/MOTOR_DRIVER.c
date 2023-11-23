/*
 * MOTOR_DRIVER.c
 *
 *  Created on: Nov 20, 2023
 *      Author: minht
 */

#include "MOTOR_DRIVER.h"
void MOTOR_driver_setupPWM(MOTOR_DRIVER_t* motor, TIM_HandleTypeDef* htimPWM, uint32_t CH1, uint32_t CH2)
{
    motor->htimPWM = htimPWM;
    motor->PWM_CH1 = CH1;
    motor->PWM_CH2 = CH2;
    HAL_TIM_PWM_Start(motor->htimPWM, motor->PWM_CH1);
    HAL_TIM_PWM_Start(motor->htimPWM, motor->PWM_CH2);
}
void MOTOR_driver_setupENCODER(MOTOR_DRIVER_t* motor, TIM_HandleTypeDef *htimENC, uint32_t CH1, uint32_t CH2)
{
    motor->htimENC = htimENC;
    motor->ENC_CH1 = CH1;
    motor->ENC_CH2 = CH2;
    HAL_TIM_Base_Start(motor->htimENC);
    HAL_TIM_Encoder_Start(motor->htimENC, motor->ENC_CH1 | motor->ENC_CH1);
}
void MOTOR_driver_rotary(MOTOR_DRIVER_t* motor, float duty)
{
    if (duty >= 0)
    {
        __HAL_TIM_SetCompare(motor->htimPWM, motor->PWM_CH1, (uint16_t)duty);
        __HAL_TIM_SetCompare(motor->htimPWM, motor->PWM_CH2, (uint16_t)0);
    }
    else
    {
        __HAL_TIM_SetCompare(motor->htimPWM, motor->PWM_CH1, (uint16_t)0);
        __HAL_TIM_SetCompare(motor->htimPWM, motor->PWM_CH2, (uint16_t)-duty);
    }
}
void MOTOR_driver_readPosAndSpeed(MOTOR_DRIVER_t* motor, float* pos, float* speed, float Ts)
{
    motor->EncCount += (int16_t)motor->htimENC->Instance->CNT; // Đọc xung encoder
    __HAL_TIM_SET_COUNTER(motor->htimENC, 0); // Reset xung encoder

    *pos = motor->pos = (float)motor->EncCount / motor->ratio; // Tính góc hiện tại của trục động cơ 
    *speed = motor->speed = (motor->pos - motor->prePos) * 1000 / Ts; // tính vận tốc xoay góc
    motor->prePos = motor->pos;
}
void MOTOR_driver_reset(MOTOR_DRIVER_t* motor)
{
    motor->pos = 0;
    motor->prePos = 0;
    motor->speed = 0;
    motor->preSpeed = 0;
    __HAL_TIM_SET_COUNTER(motor->htimENC, 0);
}
void MOTOR_driver_setRatio(MOTOR_DRIVER_t* motor, float ratio)
{
    motor->ratio = ratio;
}
