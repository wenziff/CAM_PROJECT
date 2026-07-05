#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_Init(void);
void Motor_Forward(uint16_t speed);
void Motor_Stop(void);

#endif
