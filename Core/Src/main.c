#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "motor.h"
#include "ov2640.h"
#include "spi.h"
#include "firealarm_model.h"
#include "thermal_stream.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

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
#define CONTROL_COMMAND_MAX       16U

#define THERMAL_MIN_C             5.0f
#define THERMAL_MAX_C             85.0f
#define FIRE_CLASS_ID             2
#define FIRE_STOP_DURATION_MS     5000U
#define LINE_TRACK_BASE_SPEED     500U
#define LINE_TRACK_TURN_SPEED     100U
#define LINE_TRACK_INTERVAL_MS    10U


static volatile uint8_t camera_frame_ready;
static volatile uint8_t camera_capture_error;
static volatile uint8_t remote_stop_requested;
static volatile uint8_t remote_capture_requested;
static uint8_t control_rx_byte;
static char control_command[CONTROL_COMMAND_MAX];
static uint8_t control_command_length;

static uint8_t fire_alarm_active;
static uint8_t fire_detection_armed = 1U;
static uint32_t fire_stop_tick;
static uint16_t patrol_speed_before_fire;
static uint32_t line_track_last_tick;


void SystemClock_Config(void);
static void MPU_Config(void);
static void LCD_ShowCameraTestPattern(void);

static void ShowStatusOnLCD(const char *message);
static void ShowPredictionOnLCD(const firealarm_prediction_t *prediction);
static void RunFirealarmInferenceAndDisplay(void);
static void UpdateFireResponse(const firealarm_prediction_t *prediction);
static void ServiceFireResponse(void);
static void ServiceLineTracking(void);
static void ServiceThermalStream(uint8_t *thermal_ready,
                                 uint32_t *thermal_retry_tick,
                                 uint32_t *thermal_last_poll_tick);
static float clampf_local(float value, float low, float high);
static float lerpf_local(float a, float b, float t);
static float sample_thermal_bilinear(const float *src,
                                     uint16_t src_w,
                                     uint16_t src_h,
                                     float x,
                                     float y);
static void sample_rgb565_bilinear(const uint8_t *src,
                                   uint16_t src_w,
                                   uint16_t src_h,
                                   float x,
                                   float y,
                                   float *r,
                                   float *g,
                                   float *b);
static void BuildFirealarmInput(const uint8_t *rgb_frame,
                                const float *thermal_frame,
                                float *model_input);

static uint8_t CaptureButtonPressed(void);
static uint8_t ThermalButtonPressed(void);
static HAL_StatusTypeDef Camera_CaptureFrame(void);
static void ProcessControlCommands(void);
static void ExecuteControlCommand(void);
                                
static float clampf_local(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

static float lerpf_local(float a, float b, float t)
{
  return a + (b - a) * t;
}

static float sample_thermal_bilinear(const float *src,
                                     uint16_t src_w,
                                     uint16_t src_h,
                                     float x,
                                     float y)
{
  float clamped_x;
  float clamped_y;
  int x0;
  int y0;
  int x1;
  int y1;
  float fx;
  float fy;
  float top;
  float bottom;

  clamped_x = clampf_local(x, 0.0f, (float)(src_w - 1U));
  clamped_y = clampf_local(y, 0.0f, (float)(src_h - 1U));

  x0 = (int)clamped_x;
  y0 = (int)clamped_y;
  x1 = (x0 + 1 < (int)src_w) ? (x0 + 1) : x0;
  y1 = (y0 + 1 < (int)src_h) ? (y0 + 1) : y0;
  fx = clamped_x - (float)x0;
  fy = clamped_y - (float)y0;

  top = lerpf_local(src[y0 * src_w + x0], src[y0 * src_w + x1], fx);
  bottom = lerpf_local(src[y1 * src_w + x0], src[y1 * src_w + x1], fx);
  return lerpf_local(top, bottom, fy);
}

static void sample_rgb565_bilinear(const uint8_t *src,
                                   uint16_t src_w,
                                   uint16_t src_h,
                                   float x,
                                   float y,
                                   float *r,
                                   float *g,
                                   float *b)
{
  float clamped_x;
  float clamped_y;
  int x0;
  int y0;
  int x1;
  int y1;
  float fx;
  float fy;
  float r00;
  float g00;
  float b00;
  float r01;
  float g01;
  float b01;
  float r10;
  float g10;
  float b10;
  float r11;
  float g11;
  float b11;
  uint16_t p00;
  uint16_t p01;
  uint16_t p10;
  uint16_t p11;
  const uint8_t *pix00;
  const uint8_t *pix01;
  const uint8_t *pix10;
  const uint8_t *pix11;

  clamped_x = clampf_local(x, 0.0f, (float)(src_w - 1U));
  clamped_y = clampf_local(y, 0.0f, (float)(src_h - 1U));

  x0 = (int)clamped_x;
  y0 = (int)clamped_y;
  x1 = (x0 + 1 < (int)src_w) ? (x0 + 1) : x0;
  y1 = (y0 + 1 < (int)src_h) ? (y0 + 1) : y0;
  fx = clamped_x - (float)x0;
  fy = clamped_y - (float)y0;

  pix00 = &src[(y0 * src_w + x0) * 2U];
  pix01 = &src[(y0 * src_w + x1) * 2U];
  pix10 = &src[(y1 * src_w + x0) * 2U];
  pix11 = &src[(y1 * src_w + x1) * 2U];

  p00 = (uint16_t)((((uint16_t)pix00[0]) << 8) | pix00[1]);
  p01 = (uint16_t)((((uint16_t)pix01[0]) << 8) | pix01[1]);
  p10 = (uint16_t)((((uint16_t)pix10[0]) << 8) | pix10[1]);
  p11 = (uint16_t)((((uint16_t)pix11[0]) << 8) | pix11[1]);

  r00 = (float)((p00 >> 11) & 0x1FU) * (255.0f / 31.0f);
  g00 = (float)((p00 >> 5) & 0x3FU) * (255.0f / 63.0f);
  b00 = (float)(p00 & 0x1FU) * (255.0f / 31.0f);

  r01 = (float)((p01 >> 11) & 0x1FU) * (255.0f / 31.0f);
  g01 = (float)((p01 >> 5) & 0x3FU) * (255.0f / 63.0f);
  b01 = (float)(p01 & 0x1FU) * (255.0f / 31.0f);

  r10 = (float)((p10 >> 11) & 0x1FU) * (255.0f / 31.0f);
  g10 = (float)((p10 >> 5) & 0x3FU) * (255.0f / 63.0f);
  b10 = (float)(p10 & 0x1FU) * (255.0f / 31.0f);

  r11 = (float)((p11 >> 11) & 0x1FU) * (255.0f / 31.0f);
  g11 = (float)((p11 >> 5) & 0x3FU) * (255.0f / 63.0f);
  b11 = (float)(p11 & 0x1FU) * (255.0f / 31.0f);

  r00 = lerpf_local(r00, r01, fx);
  g00 = lerpf_local(g00, g01, fx);
  b00 = lerpf_local(b00, b01, fx);
  r10 = lerpf_local(r10, r11, fx);
  g10 = lerpf_local(g10, g11, fx);
  b10 = lerpf_local(b10, b11, fx);

  *r = lerpf_local(r00, r10, fy) / 255.0f;
  *g = lerpf_local(g00, g10, fy) / 255.0f;
  *b = lerpf_local(b00, b10, fy) / 255.0f;
}

static void BuildFirealarmInput(const uint8_t *rgb_frame,
                                const float *thermal_frame,
                                float *model_input)
{
  const int plane_size = INPUT_H * INPUT_W;

  for (int y = 0; y < INPUT_H; ++y)
  {
    float rotated_y = (INPUT_H > 1) ? ((float)y * (float)(CAMERA_LCD_HEIGHT - 1U)) / (float)(INPUT_H - 1) : 0.0f;
    float thermal_y = CALIB_CROP_Y + ((INPUT_H > 1) ? ((float)y * CALIB_CROP_H) / (float)(INPUT_H - 1) : 0.0f);

    for (int x = 0; x < INPUT_W; ++x)
    {
      float rotated_x = (INPUT_W > 1) ? ((float)x * (float)(CAMERA_LCD_WIDTH - 1U)) / (float)(INPUT_W - 1) : 0.0f;
      float source_x = rotated_y;
      float source_y = (float)(CAMERA_LCD_HEIGHT - 1U) - rotated_x;
      float thermal_x = CALIB_CROP_X + ((INPUT_W > 1) ? ((float)x * CALIB_CROP_W) / (float)(INPUT_W - 1) : 0.0f);
      float r;
      float g;
      float b;
      float thermal;
      int index = y * INPUT_W + x;

      sample_rgb565_bilinear(rgb_frame,
                             CAMERA_LCD_WIDTH,
                             CAMERA_LCD_HEIGHT,
                             source_x,
                             source_y,
                             &r,
                             &g,
                             &b);
      thermal = sample_thermal_bilinear(thermal_frame,
                                        THERMAL_FRAME_WIDTH,
                                        THERMAL_FRAME_HEIGHT,
                                        thermal_x,
                                        thermal_y);

      model_input[0 * plane_size + index] = r;
      model_input[1 * plane_size + index] = g;
      model_input[2 * plane_size + index] = b;
      model_input[3 * plane_size + index] = clampf_local((thermal - THERMAL_MIN_C) / (THERMAL_MAX_C - THERMAL_MIN_C), 0.0f, 1.0f);
    }
  }
}

static void RunFirealarmInferenceAndDisplay(void)
{
  static firealarm_prediction_t prediction;
  static float model_input[INPUT_C * INPUT_H * INPUT_W];
  static float thermal_zero[THERMAL_FRAME_WIDTH * THERMAL_FRAME_HEIGHT];

  const float *thermal_frame = thermal_stream_get_latest_temperatures();
  if (thermal_stream_has_valid_frame() == 0U || thermal_frame == NULL)
  {
    thermal_frame = thermal_zero;
  }

  BuildFirealarmInput(CAMERA_FRAME_BUFFER, thermal_frame, model_input);
  firealarm_predict(model_input, &prediction);
  ShowPredictionOnLCD(&prediction);
  UpdateFireResponse(&prediction);
}

static void ServiceThermalStream(uint8_t *thermal_ready,
                                 uint32_t *thermal_retry_tick,
                                 uint32_t *thermal_last_poll_tick)
{
  const uint32_t thermal_poll_interval_ms = 200U;
  int thermal_status;

  if (thermal_ready == NULL || thermal_retry_tick == NULL || thermal_last_poll_tick == NULL)
  {
    return;
  }

  if (*thermal_ready != 0U)
  {
    if ((HAL_GetTick() - *thermal_last_poll_tick) < thermal_poll_interval_ms)
    {
      return;
    }

    *thermal_last_poll_tick = HAL_GetTick();
    thermal_status = thermal_stream_poll_and_send();
    if (thermal_status != THERMAL_STREAM_OK)
    {
      (void)thermal_stream_send_status(thermal_status);
      *thermal_ready = 0U;
      *thermal_retry_tick = HAL_GetTick() + THERMAL_RETRY_INTERVAL_MS;
    }
    return;
  }

  if ((int32_t)(HAL_GetTick() - *thermal_retry_tick) < 0)
  {
    return;
  }

  thermal_status = thermal_stream_init();
  if (thermal_status == THERMAL_STREAM_OK)
  {
    *thermal_ready = 1U;
    *thermal_last_poll_tick = HAL_GetTick();
    RunFirealarmInferenceAndDisplay();
  }
  else
  {
    (void)thermal_stream_send_status(thermal_status);
    *thermal_retry_tick = HAL_GetTick() + THERMAL_RETRY_INTERVAL_MS;
  }
}

static void ShowPredictionOnLCD(const firealarm_prediction_t *prediction)
{
  char text[32];
  float conf_percent;
  int integer_part;
  int fractional_part;

  if (prediction == NULL)
  {
    return;
  }

  /* 
   * 将浮点数转换为百分比（0.0 ~ 100.0）
   */
  conf_percent = prediction->confidence * 100.0f;
  if (conf_percent < 0.0f)
  {
    conf_percent = 0.0f;
  }
  else if (conf_percent > 100.0f)
  {
    conf_percent = 100.0f;
  }

  /* 
   * 加上 0.05f 以实现保留一位小数的四舍五入
   */
  conf_percent += 0.05f;
  integer_part = (int)conf_percent;
  fractional_part = (int)(conf_percent * 10.0f) % 10;

  /* 防止四舍五入后溢出 100% */
  if (integer_part > 100)
  {
    integer_part = 100;
    fractional_part = 0;
  }

  /* 
   * 使用 %d.%d 替代 %.1f，完全规避了浮点格式化所导致的栈溢出、
   * 8字节对齐以及 malloc 调用等底层隐患
   */
  (void)snprintf(text,
                 sizeof(text),
                 "%s %d.%d%%",
                 firealarm_class_name(prediction->class_id),
                 integer_part,
                 fractional_part);

  /* 相机画面已先写入 LCD，这里只在顶部黑边显示结果，避免覆盖图像 */
  LCD_Fill(0U, 0U, LCD_W, CAMERA_LCD_Y, BLACK);
  LCD_ShowString(20U, 20U, (const uint8_t *)text, WHITE, BLACK, 16U, 0U);
}


/* fire 类别只在非 fire -> fire 的边沿触发一次，避免持续火焰反复停车。 */
static void UpdateFireResponse(const firealarm_prediction_t *prediction)
{
  if (prediction == NULL)
  {
    return;
  }

  if (prediction->class_id != FIRE_CLASS_ID)
  {
    fire_detection_armed = 1U;
    return;
  }

  if ((fire_detection_armed != 0U) && (fire_alarm_active == 0U))
  {
    int photo_status;

    fire_detection_armed = 0U;
    fire_alarm_active = 1U;
    fire_stop_tick = HAL_GetTick();
    patrol_speed_before_fire = Motor_GetSpeed();
    Motor_Stop();
    /* LED1 接 PE13，低电平点亮。 */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    /* STM32 识别到 fire 后，只在此处上传一次当前可见光图像到 PC。 */
    photo_status = thermal_stream_send_photo_rgb565(CAMERA_FRAME_BUFFER,
                                                    CAMERA_LCD_WIDTH,
                                                    CAMERA_LCD_HEIGHT);
    if (photo_status != THERMAL_STREAM_OK)
    {
      (void)thermal_stream_send_status(photo_status);
    }

    /* 同步标记下一帧热图为 fire 快照，PC 端据此点亮报警灯并播放警报音。 */
    thermal_stream_request_snapshot();
  }
}

/* 非阻塞等待 10 秒，期间相机、热成像和串口仍可正常运行。 */
static void ServiceFireResponse(void)
{
  if ((fire_alarm_active != 0U) &&
      ((HAL_GetTick() - fire_stop_tick) >= FIRE_STOP_DURATION_MS))
  {
    fire_alarm_active = 0U;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    Motor_Forward(patrol_speed_before_fire);
  }
}

/*
 * WR-GS1/WR-GS2 灰度循迹传感器：白纸输出 1，黑色输出 0。
 * 该函数不使用 HAL_Delay，只快速读取 GPIO 并更新 PWM，避免拖慢摄像头采集和图像推理。
 */
static void ServiceLineTracking(void)
{
  uint8_t left_is_white;
  uint8_t right_is_white;
  uint32_t now = HAL_GetTick();

  if (fire_alarm_active != 0U)
  {
    return;
  }

  if ((now - line_track_last_tick) < LINE_TRACK_INTERVAL_MS)
  {
    return;
  }
  line_track_last_tick = now;

  left_is_white = (HAL_GPIO_ReadPin(LINE_LEFT_GPIO_Port,
                                    LINE_LEFT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  right_is_white = (HAL_GPIO_ReadPin(LINE_RIGHT_GPIO_Port,
                                     LINE_RIGHT_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  if ((left_is_white == 0U) && (right_is_white != 0U))
  {
    /* 左侧压到黑线，左轮降速、右轮保持巡检速度，小车向左修正。 */
    Motor_SetLeftRight(LINE_TRACK_TURN_SPEED, LINE_TRACK_BASE_SPEED);
  }
  else if ((left_is_white != 0U) && (right_is_white == 0U))
  {
    /* 右侧压到黑线，右轮降速、左轮保持巡检速度，小车向右修正。 */
    Motor_SetLeftRight(LINE_TRACK_BASE_SPEED, LINE_TRACK_TURN_SPEED);
  }
  else
  {
    /* 两侧同为白纸或同为黑色时保持直行，避免在路口/短暂丢线时停顿。 */
    Motor_Forward(LINE_TRACK_BASE_SPEED);
  }
}

static void ShowStatusOnLCD(const char *message)
{
  if (message == NULL)
  {
    return;
  }

  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(20U, 140U, (const uint8_t *)message, WHITE, BLACK, 32U, 0U);
}


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
  Motor_Init();
  if (HAL_UART_Receive_IT(&huart1, &control_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

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
    ServiceFireResponse();
    ServiceLineTracking();
    ProcessControlCommands();
    if (Camera_CaptureFrame() == HAL_OK)
    {
      /* 采集等待期间到达的停车/抓拍命令在显示本帧前立即处理。 */
      ProcessControlCommands();
      LCD_DisplayFrame(CAMERA_LCD_X, CAMERA_LCD_Y,
                       CAMERA_LCD_WIDTH, CAMERA_LCD_HEIGHT,
                       CAMERA_FRAME_BUFFER);

      RunFirealarmInferenceAndDisplay(); ////////////ceshi
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

/* 执行一条由 PC 经 ESP32-S3 转发的换行结尾 ASCII 命令。 */
static void ExecuteControlCommand(void)
{
  control_command[control_command_length] = '\0';
  if (strcmp(control_command, "STOP") == 0)
  {
    remote_stop_requested = 1U;
  }
  else if (strcmp(control_command, "CAPTURE") == 0)
  {
    remote_capture_requested = 1U;
  }
  control_command_length = 0U;
}

/* 主循环只执行中断设置的请求，避免在串口中断中操作电机。 */
static void ProcessControlCommands(void)
{
  if (remote_stop_requested != 0U)
  {
    remote_stop_requested = 0U;
    if (fire_alarm_active != 0U)
    {
      patrol_speed_before_fire = 0U;
    }
    Motor_Stop();
  }
}

/* USART1 每收到一个控制字节就立即重启中断接收，防止高速串口丢命令。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  uint8_t value;
  if (uart->Instance != USART1)
  {
    return;
  }

  value = control_rx_byte;
  if (value == '\n')
  {
    if (control_command_length != 0U)
    {
      ExecuteControlCommand();
    }
  }
  else if (value != '\r')
  {
    if (control_command_length < (CONTROL_COMMAND_MAX - 1U))
    {
      control_command[control_command_length++] = (char)value;
    }
    else
    {
      control_command_length = 0U;
    }
  }
  (void)HAL_UART_Receive_IT(&huart1, &control_rx_byte, 1U);
}

/* 过载或噪声导致接收错误时清空半条命令，并恢复下一字节接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART1)
  {
    control_command_length = 0U;
    (void)HAL_UART_Receive_IT(&huart1, &control_rx_byte, 1U);
  }
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
    ServiceFireResponse();
    ServiceLineTracking();
    ProcessControlCommands();
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
