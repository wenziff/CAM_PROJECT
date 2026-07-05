#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "ov2640.h"
#include "spi.h"
#include "thermal_stream.h"
#include "usart.h"

/* OV2640 输出 QVGA 320×240，LCD 有效区域为 240×240。
 * DCMI 从图像左右各裁掉 40 像素后，将中央 RGB565 图像写入 AXI SRAM。
 * 0x24040000 位于 DMA1 可访问的 AXI SRAM 上半区，Keil 工程已将该区域
 * 从普通变量链接空间中预留出来；不能把大帧缓冲放到 DMA1 不可访问的 DTCM。
 */
#define CAMERA_SOURCE_WIDTH       320U
#define CAMERA_LCD_WIDTH          240U
#define CAMERA_LCD_HEIGHT         240U
#define CAMERA_CROP_X_PIXELS      ((CAMERA_SOURCE_WIDTH - CAMERA_LCD_WIDTH) / 2U)
#define CAMERA_LCD_X              0U
#define CAMERA_LCD_Y              ((LCD_H - CAMERA_LCD_HEIGHT) / 2U)
#define CAMERA_BYTES_PER_PIXEL    2U
#define CAMERA_FRAME_BYTES        (CAMERA_LCD_WIDTH * CAMERA_LCD_HEIGHT * CAMERA_BYTES_PER_PIXEL)
#define CAMERA_DMA_WORDS          (CAMERA_FRAME_BYTES / 4U)
#define CAMERA_FRAME_BUFFER       ((uint8_t *)0x24040000U)
#define OV2640_EXPECTED_PID        0x2642U
#define CAMERA_FRAME_TIMEOUT_MS   1000U
#define THERMAL_RETRY_INTERVAL_MS 1000U

static volatile uint8_t camera_frame_ready;
static volatile uint8_t camera_capture_error;

void SystemClock_Config(void);
static void MPU_Config(void);
static void LCD_ShowCameraTestPattern(void);
static uint8_t CaptureButtonPressed(void);
static uint8_t ThermalButtonPressed(void);
static HAL_StatusTypeDef Camera_CaptureFrame(void);

int main(void)
{
  uint16_t camera_pid;
  uint8_t thermal_ready = 0U;
  uint32_t thermal_retry_tick = 0U;
  int thermal_status;

  MPU_Config();
  SCB_EnableICache();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  st7789_middle_init();
  LCD_ShowCameraTestPattern();
  OV2640_Init();
  camera_pid = OV2640_GetPID();

  /* PID 不正确时停在红屏，通常表示摄像头供电、复位或 SCCB 接线异常。 */
  if (camera_pid != OV2640_EXPECTED_PID)
  {
    LCD_Fill(0U, 0U, LCD_W, LCD_H, RED);
    while (1) {}
  }

  /* DCMI 的水平单位是字节：跳过 40×2 字节，保留 240×2 字节。 */
  if (HAL_DCMI_ConfigCrop(&hdcmi,
                          CAMERA_CROP_X_PIXELS * CAMERA_BYTES_PER_PIXEL,
                          0U,
                          CAMERA_LCD_WIDTH * CAMERA_BYTES_PER_PIXEL - 1U,
                          CAMERA_LCD_HEIGHT - 1U) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DCMI_EnableCrop(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }

  while (1)
  {
    if (Camera_CaptureFrame() == HAL_OK)
    {
      LCD_DisplayFrame(CAMERA_LCD_X, CAMERA_LCD_Y,
                       CAMERA_LCD_WIDTH, CAMERA_LCD_HEIGHT,
                       CAMERA_FRAME_BUFFER);

      /* KEY1 按下时冻结当前完整帧，并通过 ESP32 发送到 PC。 */
      if (CaptureButtonPressed() != 0U)
      {
        (void)thermal_stream_send_photo_rgb565(CAMERA_FRAME_BUFFER,
                                               CAMERA_LCD_WIDTH,
                                               CAMERA_LCD_HEIGHT);
      }
    }

    /* KEY2 请求保存下一张完整 MLX90640 热力图。 */
    if (ThermalButtonPressed() != 0U)
    {
      thermal_stream_request_snapshot();
    }

    /* MLX90640 使用非阻塞轮询，不等待新子页，避免拖慢 OV2640 画面。 */
    if (thermal_ready != 0U)
    {
      thermal_status = thermal_stream_poll_and_send();
      if (thermal_status != THERMAL_STREAM_OK)
      {
        (void)thermal_stream_send_status(thermal_status);
        thermal_ready = 0U;
        thermal_retry_tick = HAL_GetTick() + THERMAL_RETRY_INTERVAL_MS;
      }
    }
    else if ((int32_t)(HAL_GetTick() - thermal_retry_tick) >= 0)
    {
      thermal_status = thermal_stream_init();
      if (thermal_status == THERMAL_STREAM_OK)
      {
        thermal_ready = 1U;
      }
      else
      {
        (void)thermal_stream_send_status(thermal_status);
        thermal_retry_tick = HAL_GetTick() + THERMAL_RETRY_INTERVAL_MS;
      }
    }
  }
}

/*
 * 上电先显示 1 秒固定彩条，用于快速区分故障位置：
 * 彩条正常而相机画面花屏，说明 LCD/SPI 正常，应继续检查 DCMI 时序；
 * 彩条本身也花屏，则优先检查 LCD 接线、方向参数和 SPI 时序。
 */
static void LCD_ShowCameraTestPattern(void)
{
  uint32_t x;
  uint32_t y;
  uint32_t index;
  uint16_t color;

  for (y = 0U; y < CAMERA_LCD_HEIGHT; y++)
  {
    for (x = 0U; x < CAMERA_LCD_WIDTH; x++)
    {
      if (x < (CAMERA_LCD_WIDTH / 4U))
      {
        color = RED;
      }
      else if (x < (CAMERA_LCD_WIDTH / 2U))
      {
        color = GREEN;
      }
      else if (x < ((CAMERA_LCD_WIDTH * 3U) / 4U))
      {
        color = BLUE;
      }
      else
      {
        color = WHITE;
      }

      index = (y * CAMERA_LCD_WIDTH + x) * CAMERA_BYTES_PER_PIXEL;
      CAMERA_FRAME_BUFFER[index] = (uint8_t)(color >> 8);
      CAMERA_FRAME_BUFFER[index + 1U] = (uint8_t)color;
    }
  }

  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_DisplayFrame(CAMERA_LCD_X, CAMERA_LCD_Y,
                   CAMERA_LCD_WIDTH, CAMERA_LCD_HEIGHT,
                   CAMERA_FRAME_BUFFER);
  HAL_Delay(1000U);
}

/* KEY1 接 PA0，按下时接地；消抖并确保长按只拍摄一次。 */
static uint8_t CaptureButtonPressed(void)
{
  static uint8_t latched;
  uint8_t pressed = HAL_GPIO_ReadPin(BTN_CAPTURE_GPIO_Port,
                                    BTN_CAPTURE_Pin) == GPIO_PIN_RESET;

  if (pressed == 0U)
  {
    latched = 0U;
    return 0U;
  }
  if (latched != 0U)
  {
    return 0U;
  }

  HAL_Delay(20U);
  if (HAL_GPIO_ReadPin(BTN_CAPTURE_GPIO_Port,
                       BTN_CAPTURE_Pin) == GPIO_PIN_RESET)
  {
    latched = 1U;
    return 1U;
  }
  return 0U;
}

/* KEY2 接 PC1，按下时接地；消抖并确保长按只保存一次热图。 */
static uint8_t ThermalButtonPressed(void)
{
  static uint8_t latched;
  uint8_t pressed = HAL_GPIO_ReadPin(BTN_THERMAL_GPIO_Port,
                                    BTN_THERMAL_Pin) == GPIO_PIN_RESET;

  if (pressed == 0U)
  {
    latched = 0U;
    return 0U;
  }
  if (latched != 0U)
  {
    return 0U;
  }

  HAL_Delay(20U);
  if (HAL_GPIO_ReadPin(BTN_THERMAL_GPIO_Port,
                       BTN_THERMAL_Pin) == GPIO_PIN_RESET)
  {
    latched = 1U;
    return 1U;
  }
  return 0U;
}

/* 启动一次快照采集，并等待帧中断。LCD 刷新期间摄像头 DMA 不工作，避免撕裂。 */
static HAL_StatusTypeDef Camera_CaptureFrame(void)
{
  uint32_t start_tick = HAL_GetTick();

  camera_frame_ready = 0U;
  camera_capture_error = 0U;
  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                         (uint32_t)CAMERA_FRAME_BUFFER,
                         CAMERA_DMA_WORDS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  while ((camera_frame_ready == 0U) && (camera_capture_error == 0U))
  {
    if ((HAL_GetTick() - start_tick) >= CAMERA_FRAME_TIMEOUT_MS)
    {
      (void)HAL_DCMI_Stop(&hdcmi);
      return HAL_TIMEOUT;
    }
  }

  return camera_capture_error == 0U ? HAL_OK : HAL_ERROR;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *dcmi)
{
  if (dcmi->Instance == DCMI)
  {
    camera_frame_ready = 1U;
  }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *dcmi)
{
  if (dcmi->Instance == DCMI)
  {
    camera_capture_error = 1U;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
