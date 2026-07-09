#include "motor.h"
#include "tim.h"
void Motor_SetLeftRight(uint16_t left_speed, uint16_t right_speed);
void Motor_Init(void)
{
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  Motor_Stop();
}

void Motor_Forward(uint16_t speed)
{
      Motor_SetLeftRight(speed, speed);
}
void Motor_SetLeftRight(uint16_t left_speed, uint16_t right_speed)
{
  if (left_speed > 999U)
  {
    left_speed = 999U;
  }
  if (right_speed > 999U)
  {
    right_speed = 999U;
  }
  
    /*
   * PB8/TIM4_CH3 和 PB9/TIM4_CH4 为当前工程已有的两路电机 PWM。
   * 若实车左右轮相反，只需要在这里互换两个通道的 compare 值。
   */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_speed);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_speed);
}
void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

uint16_t Motor_GetSpeed(void)
{
  return (uint16_t)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
}
