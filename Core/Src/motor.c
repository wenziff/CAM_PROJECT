#include "motor.h"
#include "tim.h"

void Motor_Init(void)
{
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  Motor_Stop();
}

void Motor_Forward(uint16_t speed)
{
  if (speed > 999) speed = 999;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
}

void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}
