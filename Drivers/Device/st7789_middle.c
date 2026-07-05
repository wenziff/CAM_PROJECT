#include "st7789_middle.h"

#include "st7789_driver.h"
#include "spi.h"

 
int st7789_init(void)
{	
	LCD_BLK_Set();
	LCD_CS_Set();
	LCD_DC_Set();
	LCD_RES_Set();
	//SPI1_Init();
	
	return 0;

}
uint8_t spi_read_write_one_byte (uint8_t data )
{
	//SPI1_ReadWriteByte(data);
	HAL_SPI_Transmit(&hspi1,&data,1,100);
	return 0;
}
uint8_t spi_read_write_len_bytes(uint8_t *data ,uint16_t len)
{
	return 0;
}
void st7789_cs_high(void)
{
	LCD_CS_Set();
}
void st7789_cs_low(void)
{
	LCD_CS_Clr();
}
void st7789_dc_high(void)
{
	LCD_DC_Set();
}
void st7789_dc_low(void)
{
	LCD_DC_Clr();
}
void st7789_res_high(void)
{
	LCD_RES_Set();
}
void st7789_res_low(void)
{
	LCD_RES_Clr();
}
void st7789_blk_high(void)
{
	LCD_BLK_Set();
}
void st7789_blk_low(void)
{
	LCD_BLK_Clr();
}
void st7789_delay_ms     (unsigned short ms)
{
	HAL_Delay(ms);
}


struct st7789_operations st7789_oper = {
	.st7789_init=st7789_init,
	.spi_read_write_one_byte=spi_read_write_one_byte,
	.spi_read_write_len_bytes=spi_read_write_len_bytes,
	.st7789_cs_high=st7789_cs_high,
	.st7789_cs_low=st7789_cs_low,
	.st7789_dc_high=st7789_dc_high,
	.st7789_dc_low=st7789_dc_low,
	.st7789_res_high=st7789_res_high,
	.st7789_res_low=st7789_res_low,
	.st7789_blk_high=st7789_blk_high,
	.st7789_blk_low=st7789_blk_low,
	.delay_ms=st7789_delay_ms,
};

void st7789_middle_init()
{
	st7789_operation_register(&st7789_oper);
	LCD_Init();
}

void st7789_example()
{
	//初始化软件SPI

	//注册ST7789操作函数
	st7789_operation_register(&st7789_oper);
	LCD_Init();
	LCD_Fill(0, 0, LCD_W, LCD_H, GREEN);
	HAL_Delay(1000);
    while (1)
    {
		LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
		HAL_Delay(1000);
		LCD_Fill(0, 0, LCD_W, LCD_H, RED);
		HAL_Delay(1000);
		LCD_Fill(0, 0, LCD_W, LCD_H, GREEN);
		HAL_Delay(1000);
		LCD_Fill(0, 0, LCD_W, LCD_H, BLUE);
		HAL_Delay(1000);
		LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
		HAL_Delay(1000);
    }
	
//	LCD_Fill_WITH_BUFFER(30,40,LCD_W,LCD_H,&color);    
//	HAL_Delay(1000);
//    while (1)
//    {
//		color = GREEN;
//		LCD_Fill_WITH_BUFFER(0,0,LCD_W,LCD_H,&color);    
//		HAL_Delay(1000);
//		color = BLACK;
//		LCD_Fill_WITH_BUFFER(0,0,LCD_W,LCD_H,&color);  
//		HAL_Delay(1000);
//		color = BLUE;
//        LCD_Fill_WITH_BUFFER(0,0,LCD_W,LCD_H,&color);  
//		HAL_Delay(1000);
//		color = WHITE;
//		LCD_Fill_WITH_BUFFER(0,0,LCD_W,LCD_H,&color);  
//		HAL_Delay(1000);
//		LCD_Fill_WITH_BUFFER(0,0,LCD_W,LCD_H,&color);  
//		HAL_Delay(1000);
//    }
	
}
