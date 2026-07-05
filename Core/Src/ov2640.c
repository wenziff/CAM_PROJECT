#include "ov2640.h"
#include "ov2640_cfg.h"
#include "i2c.h"

#define OV2640_ADDR 0x60

static void wrReg(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c2, OV2640_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static uint8_t rdReg(uint8_t reg)
{
  uint8_t v = 0;
  HAL_I2C_Mem_Read(&hi2c2, OV2640_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
  return v;
}

void OV2640_Init(void)
{
  uint32_t i;

  {
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_1;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_GPIO_Init(GPIOD, &g);
  }

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(10);

  /*
   * 先执行传感器软复位，再写入完整配置表。
   * 仅做硬件复位时，部分模组上电后的 DSP 状态可能不一致。
   */
  wrReg(0xFF, 0x01);
  wrReg(0x12, 0x80);
  HAL_Delay(10);

  for (i = 0; i < sizeof(ov2640_qvga_cfg) / sizeof(ov2640_qvga_cfg[0]); i++)
    wrReg(ov2640_qvga_cfg[i][0], ov2640_qvga_cfg[i][1]);

  /*
   * 配置表已经完成传感器窗口与 320×240 缩放，不能再单独改 COM7，
   * 否则传感器分辨率与 DSP 窗口不一致，会表现为规则竖纹或花屏。
   * COM10=0：HREF/VSYNC 保持默认极性，数据在 PCLK 下降沿更新，
   * 因此 STM32 DCMI 使用 PCLK 上升沿采样。
   */
  wrReg(0xFF, 0x01);
  wrReg(0x15, 0x00);

  /* 按 OV2640 推荐顺序复位 DVP，再切换为 RGB565 高字节优先输出。 */
  wrReg(0xFF, 0x00);
  wrReg(0xE0, 0x04);
  wrReg(0xDA, 0x08);
  wrReg(0xD7, 0x03);
  wrReg(0xE1, 0x77);
  wrReg(0xE0, 0x00);
  HAL_Delay(30);
}

uint16_t OV2640_GetPID(void)
{
  uint16_t pid;
  wrReg(0xFF, 0x01);
  pid  = (uint16_t)rdReg(OV2640_SENSOR_PIDH) << 8;
  pid |= rdReg(OV2640_SENSOR_PIDL);
  return pid;
}
