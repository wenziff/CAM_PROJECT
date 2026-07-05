#include "lcd.h"

void LCD_DisplayFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      const uint8_t *data)
{
  /* DCMI 裁剪后得到连续的 280×240 RGB565 数据，可整帧高速发送。 */
  LCD_Address_Set(x, y, x + w - 1U, y + h - 1U);
  LCD_WR_DATA8_LEN((uint8_t *)data, (uint32_t)w * h * 2U);
  LCD_CS_Set();
}
