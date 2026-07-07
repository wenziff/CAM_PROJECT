#ifndef THERMAL_STREAM_H
#define THERMAL_STREAM_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define THERMAL_FRAME_WIDTH          32U
#define THERMAL_FRAME_HEIGHT         24U
#define THERMAL_PIXEL_COUNT          (THERMAL_FRAME_WIDTH * THERMAL_FRAME_HEIGHT)
#define THERMAL_PAYLOAD_SIZE         (THERMAL_PIXEL_COUNT * 2U)
#define THERMAL_PROTOCOL_HEADER_SIZE 22U
#define THERMAL_PROTOCOL_FRAME_SIZE  (THERMAL_PROTOCOL_HEADER_SIZE + THERMAL_PAYLOAD_SIZE + 2U)
enum {
    THERMAL_STREAM_OK = 0,
    THERMAL_STREAM_ERR_SENSOR_NOT_FOUND = -100,
    THERMAL_STREAM_ERR_DATA_TIMEOUT = -101,
    THERMAL_STREAM_ERR_UART = -102,
    THERMAL_STREAM_ERR_ARGUMENT = -103
};
int thermal_stream_init(void);
int thermal_stream_capture_and_send(void);
/* 非阻塞轮询：没有新子页时立即返回，适合与 OV2640 LCD 刷新交替运行。 */
int thermal_stream_poll_and_send(void);
int thermal_stream_send_status(int32_t status_code);
/* 获取最近一次完整热图的温度数组；未准备好时返回 NULL。 */
const float *thermal_stream_get_latest_temperatures(void);
uint8_t thermal_stream_has_valid_frame(void);
/* KEY2 调用此函数后，下一张完整热图将标记为需要保存的快照。 */
void thermal_stream_request_snapshot(void);
/* 将 RGB565 照片拆成多个 CRC 分片，通过同一 USART1 链路发送。 */
int thermal_stream_send_photo_rgb565(const uint8_t *image,
                                     uint16_t width, uint16_t height);
#ifdef __cplusplus
}
#endif
#endif
