#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_Init(void);
void Motor_Forward(uint16_t speed);
void Motor_SetLeftRight(uint16_t left_speed, uint16_t right_speed);
void Motor_Stop(void);
uint16_t Motor_GetSpeed(void);

#endif
