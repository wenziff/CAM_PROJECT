#include "sccb.h"

#define SCCB_GPIOx GPIOB
#define SIO_C_PIN GPIO_PIN_10
#define SIO_D_PIN GPIO_PIN_11

static void sccb_delay(void) { volatile uint32_t i; for (i = 0; i < 2400; i++); }

static void sio_d_input(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = SIO_D_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCCB_GPIOx, &g);
}

static void sio_d_output(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = SIO_D_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCCB_GPIOx, &g);
}

static void sccb_start(void)
{
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_SET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_RESET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_RESET);
  sccb_delay();
}

static void sccb_stop(void)
{
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_RESET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_SET);
  sccb_delay();
}

static uint8_t sccb_write_byte(uint8_t byte)
{
  uint8_t i, bit;
  for (i = 0; i < 8; i++) {
    if (byte & 0x80) HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_SET);
    else             HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_RESET);
    byte <<= 1;
    sccb_delay();
    HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
    sccb_delay();
    HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_RESET);
    sccb_delay();
  }
  sio_d_input();
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
  bit = HAL_GPIO_ReadPin(SCCB_GPIOx, SIO_D_PIN);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_RESET);
  sccb_delay();
  sio_d_output();
  return bit;
}

static uint8_t sccb_read_byte(void)
{
  uint8_t byte = 0, i;
  sio_d_input();
  for (i = 0; i < 8; i++) {
    sccb_delay();
    HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(SCCB_GPIOx, SIO_D_PIN))
      byte |= (0x80 >> i);
    sccb_delay();
    HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_RESET);
    sccb_delay();
  }
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_D_PIN, GPIO_PIN_SET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_SET);
  sccb_delay();
  HAL_GPIO_WritePin(SCCB_GPIOx, SIO_C_PIN, GPIO_PIN_RESET);
  sccb_delay();
  sio_d_output();
  return byte;
}

void sccb_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
  sccb_start();
  sccb_write_byte(dev_addr);
  sccb_write_byte(reg);
  sccb_write_byte(data);
  sccb_stop();
}

uint8_t sccb_read_reg(uint8_t dev_addr, uint8_t reg)
{
  uint8_t data;
  sccb_start();
  sccb_write_byte(dev_addr);
  sccb_write_byte(reg);
  sccb_stop();
  sccb_start();
  sccb_write_byte(dev_addr | 0x01);
  data = sccb_read_byte();
  sccb_stop();
  return data;
}
