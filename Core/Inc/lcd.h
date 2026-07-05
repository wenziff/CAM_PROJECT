#ifndef __LCD_H__
#define __LCD_H__

#include "main.h"
#include "st7789_middle.h"
#include "st7789_driver.h"

/* 将一块连续存储的 RGB565 图像一次性写入 LCD。 */
void LCD_DisplayFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      const uint8_t *data);

#endif
