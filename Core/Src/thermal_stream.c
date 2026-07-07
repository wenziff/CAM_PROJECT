#include "thermal_stream.h"

#include <math.h>
#include <string.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "i2c.h"
#include "usart.h"

#define MLX90640_ADDRESS          0x33U
#define MLX90640_REFRESH_RATE_8HZ 0x04U
#define MLX90640_RESOLUTION_18BIT 0x03U
#define MLX90640_EMISSIVITY       0.95f
#define MLX90640_TA_SHIFT         8.0f
#define MLX90640_READY_TIMEOUT_MS 500U
#define PROTOCOL_VERSION          1U
#define FRAME_TYPE_TEMPERATURE    0x01U
#define FRAME_TYPE_PHOTO_CHUNK    0x02U
#define FRAME_TYPE_THERMAL_SNAPSHOT 0x03U
#define FRAME_TYPE_STATUS         0x7FU
#define PIXEL_FORMAT_CENTI_C_I16  0x01U
#define STATUS_FORMAT_I32         0x01U
#define MAX_FRAME_SIZE            THERMAL_PROTOCOL_FRAME_SIZE
#define PHOTO_CHUNK_SIZE          4096U
#define RGB565_BYTES_PER_PIXEL    2U

static const uint8_t frame_magic[4] = {'M', 'L', 'X', '4'};
static paramsMLX90640 mlx_params;
static uint16_t mlx_eeprom[832];
static uint16_t mlx_frame[834];
static float temperatures[THERMAL_PIXEL_COUNT];
static uint8_t tx_frame[MAX_FRAME_SIZE];
static uint32_t frame_sequence;
static uint32_t photo_sequence;
static int mlx_mode = 1;
static uint8_t poll_subpage_mask;
static uint8_t snapshot_requested;
static uint8_t latest_frame_valid;

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void put_i16_le(uint8_t *dst, int16_t value)
{
    put_u16_le(dst, (uint16_t)value);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static uint16_t crc16_update(uint16_t crc, const uint8_t *data,
                             uint32_t length)
{
    uint16_t i;
    while (length-- != 0U) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0U; i < 8U; ++i) {
            crc = (crc & 0x8000U) != 0U
                    ? (uint16_t)((crc << 1) ^ 0x1021U)
                    : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    return crc16_update(0xFFFFU, data, length);
}

static int wait_for_new_data(void)
{
    uint16_t status_register = 0U;
    uint32_t start = HAL_GetTick();
    int status;

    do {
        status = MLX90640_I2CRead(MLX90640_ADDRESS, 0x8000U, 1U,
                                  &status_register);
        if (status != 0) return status;
        if ((status_register & 0x0008U) != 0U) return THERMAL_STREAM_OK;
        HAL_Delay(1U);
    } while ((HAL_GetTick() - start) < MLX90640_READY_TIMEOUT_MS);
    return THERMAL_STREAM_ERR_DATA_TIMEOUT;
}

static int acquire_temperature_frame(void)
{
    uint8_t subpage_mask = 0U;
    uint8_t attempts = 0U;

    while ((subpage_mask != 0x03U) && (attempts < 6U)) {
        float ambient;
        float reflected;
        int subpage;
        int status = wait_for_new_data();

        if (status != THERMAL_STREAM_OK) return status;
        subpage = MLX90640_GetFrameData(MLX90640_ADDRESS, mlx_frame);
        if (subpage < 0) return subpage;

        ambient = MLX90640_GetTa(mlx_frame, &mlx_params);
        reflected = ambient - MLX90640_TA_SHIFT;
        MLX90640_CalculateTo(mlx_frame, &mlx_params,
                            MLX90640_EMISSIVITY, reflected, temperatures);
        subpage_mask |= (uint8_t)(1U << ((uint8_t)subpage & 1U));
        ++attempts;
    }
    if (subpage_mask != 0x03U) return THERMAL_STREAM_ERR_DATA_TIMEOUT;

    MLX90640_BadPixelsCorrection(mlx_params.brokenPixels, temperatures,
                                 mlx_mode, &mlx_params);
    MLX90640_BadPixelsCorrection(mlx_params.outlierPixels, temperatures,
                                 mlx_mode, &mlx_params);
    latest_frame_valid = 1U;
    return THERMAL_STREAM_OK;
}

static int16_t temperature_to_centi(float temperature)
{
    float scaled;
    if (!isfinite(temperature)) return INT16_MIN;
    scaled = temperature * 100.0f;
    if (scaled > 32767.0f) return INT16_MAX;
    if (scaled < -32767.0f) return -32767;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint16_t build_header(uint8_t frame_type, uint8_t pixel_format,
                             uint8_t width, uint8_t height,
                             uint16_t payload_length,
                             int16_t minimum, int16_t maximum)
{
    memcpy(tx_frame, frame_magic, sizeof(frame_magic));
    tx_frame[4] = PROTOCOL_VERSION;
    tx_frame[5] = frame_type;
    tx_frame[6] = width;
    tx_frame[7] = height;
    put_u32_le(&tx_frame[8], frame_sequence);
    put_u32_le(&tx_frame[12], HAL_GetTick());
    put_u16_le(&tx_frame[16], payload_length);
    put_i16_le(&tx_frame[18], minimum);
    put_i16_le(&tx_frame[20], maximum);
    (void)pixel_format;
    return THERMAL_PROTOCOL_HEADER_SIZE;
}

static int transmit_frame(uint16_t frame_length)
{
    uint16_t crc = crc16_ccitt_false(&tx_frame[4],
                                     (uint16_t)(frame_length - 6U));
    put_u16_le(&tx_frame[frame_length - 2U], crc);
    return HAL_UART_Transmit(&huart1, tx_frame, frame_length, 100U) == HAL_OK
           ? THERMAL_STREAM_OK : THERMAL_STREAM_ERR_UART;
}

static int transmit_temperature_frame(void)
{
    uint8_t frame_type = snapshot_requested != 0U
                         ? FRAME_TYPE_THERMAL_SNAPSHOT
                         : FRAME_TYPE_TEMPERATURE;
    int16_t minimum = INT16_MAX;
    int16_t maximum = INT16_MIN;
    uint16_t i;
    uint16_t offset;
    int status;

    offset = build_header(frame_type,
                          PIXEL_FORMAT_CENTI_C_I16,
                          THERMAL_FRAME_WIDTH, THERMAL_FRAME_HEIGHT,
                          THERMAL_PAYLOAD_SIZE, 0, 0);
    for (i = 0U; i < THERMAL_PIXEL_COUNT; ++i) {
        int16_t value = temperature_to_centi(temperatures[i]);
        put_i16_le(&tx_frame[offset + (i * 2U)], value);
        if (value != INT16_MIN) {
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
        }
    }
    if (minimum == INT16_MAX) {
        minimum = INT16_MIN;
        maximum = INT16_MIN;
    }
    put_i16_le(&tx_frame[18], minimum);
    put_i16_le(&tx_frame[20], maximum);

    status = transmit_frame(THERMAL_PROTOCOL_FRAME_SIZE);
    if (status == THERMAL_STREAM_OK) {
        ++frame_sequence;
        if (frame_type == FRAME_TYPE_THERMAL_SNAPSHOT) {
            snapshot_requested = 0U;
        }
    }
    return status;
}

int thermal_stream_init(void)
{
    int status;
    if (HAL_I2C_IsDeviceReady(&hi2c3, MLX90640_ADDRESS << 1,
                              3U, 100U) != HAL_OK) {
        return THERMAL_STREAM_ERR_SENSOR_NOT_FOUND;
    }
    status = MLX90640_DumpEE(MLX90640_ADDRESS, mlx_eeprom);
    if (status != 0) return status;
    status = MLX90640_ExtractParameters(mlx_eeprom, &mlx_params);
    if (status != 0) return status;
    status = MLX90640_SetResolution(MLX90640_ADDRESS,
                                    MLX90640_RESOLUTION_18BIT);
    if (status != 0) return status;
    status = MLX90640_SetRefreshRate(MLX90640_ADDRESS,
                                     MLX90640_REFRESH_RATE_8HZ);
    if (status != 0) return status;
    status = MLX90640_SetChessMode(MLX90640_ADDRESS);
    if (status != 0) return status;
    mlx_mode = MLX90640_GetCurMode(MLX90640_ADDRESS);
    poll_subpage_mask = 0U;
    latest_frame_valid = 0U;
    return mlx_mode < 0 ? mlx_mode : THERMAL_STREAM_OK;
}

int thermal_stream_capture_and_send(void)
{
    int status = acquire_temperature_frame();

    if (status != THERMAL_STREAM_OK) return status;
    return transmit_temperature_frame();
}

int thermal_stream_poll_and_send(void)
{
    uint16_t status_register = 0U;
    float ambient;
    float reflected;
    int subpage;
    int status;

    /* 只读取一次状态寄存器；数据未就绪时不等待，让主循环继续刷新 LCD。 */
    status = MLX90640_I2CRead(MLX90640_ADDRESS, 0x8000U, 1U,
                              &status_register);
    if (status != 0) return status;
    if ((status_register & 0x0008U) == 0U) return THERMAL_STREAM_OK;

    subpage = MLX90640_GetFrameData(MLX90640_ADDRESS, mlx_frame);
    if (subpage < 0) return subpage;

    ambient = MLX90640_GetTa(mlx_frame, &mlx_params);
    reflected = ambient - MLX90640_TA_SHIFT;
    MLX90640_CalculateTo(mlx_frame, &mlx_params,
                        MLX90640_EMISSIVITY, reflected, temperatures);
    poll_subpage_mask |= (uint8_t)(1U << ((uint8_t)subpage & 1U));

    /* 两个棋盘子页都更新后，才校正坏点并向 ESP32 发送完整 32×24 帧。 */
    if (poll_subpage_mask != 0x03U) return THERMAL_STREAM_OK;
    poll_subpage_mask = 0U;
    MLX90640_BadPixelsCorrection(mlx_params.brokenPixels, temperatures,
                                 mlx_mode, &mlx_params);
    MLX90640_BadPixelsCorrection(mlx_params.outlierPixels, temperatures,
                                 mlx_mode, &mlx_params);
    latest_frame_valid = 1U;
    return transmit_temperature_frame();
}

const float *thermal_stream_get_latest_temperatures(void)
{
    return latest_frame_valid != 0U ? temperatures : (const float *)0;
}

uint8_t thermal_stream_has_valid_frame(void)
{
    return latest_frame_valid;
}

void thermal_stream_request_snapshot(void)
{
    snapshot_requested = 1U;
}

static int transmit_photo_chunk(const uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t width, uint8_t height,
                                uint32_t timestamp_ms,
                                uint16_t chunk_index,
                                uint16_t chunk_count)
{
    uint8_t crc_bytes[2];
    uint16_t crc;

    memcpy(tx_frame, frame_magic, sizeof(frame_magic));
    tx_frame[4] = PROTOCOL_VERSION;
    tx_frame[5] = FRAME_TYPE_PHOTO_CHUNK;
    tx_frame[6] = width;
    tx_frame[7] = height;
    put_u32_le(&tx_frame[8], photo_sequence);
    put_u32_le(&tx_frame[12], timestamp_ms);
    put_u16_le(&tx_frame[16], payload_length);
    put_u16_le(&tx_frame[18], chunk_index);
    put_u16_le(&tx_frame[20], chunk_count);

    crc = crc16_update(0xFFFFU, &tx_frame[4],
                       THERMAL_PROTOCOL_HEADER_SIZE - 4U);
    crc = crc16_update(crc, payload, payload_length);
    put_u16_le(crc_bytes, crc);

    if (HAL_UART_Transmit(&huart1, tx_frame,
                          THERMAL_PROTOCOL_HEADER_SIZE, 100U) != HAL_OK ||
        HAL_UART_Transmit(&huart1, (uint8_t *)payload,
                          payload_length, 1000U) != HAL_OK ||
        HAL_UART_Transmit(&huart1, crc_bytes,
                          sizeof(crc_bytes), 100U) != HAL_OK) {
        return THERMAL_STREAM_ERR_UART;
    }
    return THERMAL_STREAM_OK;
}

int thermal_stream_send_photo_rgb565(const uint8_t *image,
                                     uint16_t width, uint16_t height)
{
    uint32_t total_length;
    uint32_t offset = 0U;
    uint32_t timestamp_ms;
    uint16_t chunk_count;
    uint16_t chunk_index;

    /* 协议头中的宽高各占一个字节，当前照片尺寸必须不超过 255。 */
    if (image == NULL || width == 0U || height == 0U ||
        width > 255U || height > 255U) {
        return THERMAL_STREAM_ERR_ARGUMENT;
    }

    total_length = (uint32_t)width * height * RGB565_BYTES_PER_PIXEL;
    chunk_count = (uint16_t)((total_length + PHOTO_CHUNK_SIZE - 1U) /
                             PHOTO_CHUNK_SIZE);
    timestamp_ms = HAL_GetTick();

    for (chunk_index = 0U; chunk_index < chunk_count; ++chunk_index) {
        uint32_t remaining = total_length - offset;
        uint16_t chunk_length = (uint16_t)(remaining > PHOTO_CHUNK_SIZE
                                          ? PHOTO_CHUNK_SIZE : remaining);
        int status = transmit_photo_chunk(image + offset, chunk_length,
                                          (uint8_t)width, (uint8_t)height,
                                          timestamp_ms, chunk_index,
                                          chunk_count);
        if (status != THERMAL_STREAM_OK) return status;
        offset += chunk_length;
    }

    ++photo_sequence;
    return THERMAL_STREAM_OK;
}

int thermal_stream_send_status(int32_t status_code)
{
    uint16_t offset = build_header(FRAME_TYPE_STATUS, STATUS_FORMAT_I32,
                                   0U, 0U, 4U, 0, 0);
    put_u32_le(&tx_frame[offset], (uint32_t)status_code);
    return transmit_frame((uint16_t)(THERMAL_PROTOCOL_HEADER_SIZE + 6U));
}
