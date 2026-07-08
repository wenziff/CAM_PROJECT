/*
 * MLX90640 热图与 OV2640 照片桥接程序：STM32 -> ESP32-S3 -> PC。
 *
 * 接线：
 *   STM32 PA9（USART1_TX）-> ESP32-S3 GPIO17（UART2_RX）
 *   STM32 PA10（USART1_RX）<- ESP32-S3 GPIO18（UART2_TX，可选）
 *   STM32 GND <-> ESP32-S3 GND
 *
 * STM32 与 ESP32-S3 之间使用 921600、8N1 串口通信。
 * ESP32-S3 以 STA 模式连接路由器，并主动连接 PC 的 TCP 8888 端口，
 * 向 PC 转发通过 CRC 校验的完整帧，并把 PC 控制命令反向转发给 STM32。
 */
#include <WiFi.h>

// STA 模式使用的路由器信息，请按现场 Wi-Fi 修改这两项。
static const char *WIFI_SSID = "";
static const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char *DEVICE_HOSTNAME = "mlx90640-cam";
static const char *PC_IP = "10.211.44.196";
static const uint16_t TCP_PORT = 8888;

// UART2 引脚及波特率。GPIO17 接 STM32 的 PA9/TX。
static const int UART_RX_PIN = 17;
static const int UART_TX_PIN = 18;
static const uint32_t UART_BAUD = 921600;

// STM32 数据帧协议参数。
// 温度帧最大 1536 字节；OV2640 照片按最多 4096 字节的数据分片发送。
static const uint8_t MAGIC[4] = {'M', 'L', 'X', '4'};
static const size_t HEADER_SIZE = 22;
static const size_t CRC_SIZE = 2;
static const size_t MAX_PAYLOAD_SIZE = 4096;
static const size_t MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

// 接收缓存可容纳 3 帧，用于处理串口分包、粘包及短时间的网络阻塞。
static const size_t RX_BUFFER_SIZE = MAX_FRAME_SIZE * 3;

// ESP32-S3 作为 TCP 客户端，主动连接 PC 上运行的接收服务。
WiFiClient client;

// STM32 串口数据缓存及运行统计。
static uint8_t rxBuffer[RX_BUFFER_SIZE];
static size_t rxLength = 0;
static uint32_t validFrames = 0;
static uint32_t crcErrors = 0;
static uint32_t droppedFrames = 0;
static uint32_t controlBytes = 0;
static uint32_t lastStatsMs = 0;
static uint32_t lastWiFiRetryMs = 0;
static uint32_t lastTcpRetryMs = 0;

// 从字节流读取一个小端序 uint16_t。
static uint16_t readU16Le(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

// CRC16/CCITT-FALSE：初值 0xFFFF，多项式 0x1021。
// 校验范围与 STM32、PC 端完全一致：协议版本字段至 payload 末尾。
static uint16_t crc16CcittFalse(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length-- != 0) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// 从缓存头部删除指定字节数，并把剩余数据移动到缓存起始位置。
static void discardPrefix(size_t count) {
  if (count >= rxLength) {
    rxLength = 0;
    return;
  }
  memmove(rxBuffer, rxBuffer + count, rxLength - count);
  rxLength -= count;
}

// 确保一帧数据全部写入 TCP，避免一次 write() 只发送部分字节。
// 连接异常或超过 1000 ms 仍未发送完成时返回 false。
static bool sendAll(const uint8_t *data, size_t length) {
  size_t sent = 0;
  uint32_t start = millis();
  while (sent < length && client && client.connected()) {
    size_t written = client.write(data + sent, length - sent);
    if (written == 0) {
      if (millis() - start > 1000) return false;
      delay(1);
      continue;
    }
    sent += written;
  }
  return sent == length;
}

// 从串口缓存中持续提取完整帧。
// 支持自动搜索魔数、等待残帧、过滤非法长度并校验 CRC。
static void processFrames() {
  for (;;) {
    size_t magicPosition = 0;
    while ((magicPosition + sizeof(MAGIC) <= rxLength) &&
           memcmp(rxBuffer + magicPosition, MAGIC, sizeof(MAGIC)) != 0) {
      ++magicPosition;
    }

    // 没找到完整魔数时只保留最后 3 字节，以防魔数横跨两次串口读取。
    if (magicPosition + sizeof(MAGIC) > rxLength) {
      if (rxLength > sizeof(MAGIC) - 1) {
        size_t keep = sizeof(MAGIC) - 1;
        memmove(rxBuffer, rxBuffer + rxLength - keep, keep);
        rxLength = keep;
      }
      return;
    }

    // 丢弃魔数之前的噪声字节，使缓存从 MLX4 开始。
    if (magicPosition != 0) discardPrefix(magicPosition);
    if (rxLength < HEADER_SIZE) return;

    // 当前只接受版本 1；版本不正确时向后移动一个字节重新同步。
    if (rxBuffer[4] != 1) {
      discardPrefix(1);
      ++droppedFrames;
      continue;
    }

    // payload 长度位于帧头偏移 16，采用小端序。
    uint16_t payloadLength = readU16Le(rxBuffer + 16);
    if (payloadLength > MAX_PAYLOAD_SIZE) {
      discardPrefix(1);
      ++droppedFrames;
      continue;
    }

    // 串口数据不足一整帧时先返回，等待下一批数据。
    size_t frameLength = HEADER_SIZE + payloadLength + CRC_SIZE;
    if (rxLength < frameLength) return;

    uint16_t receivedCrc = readU16Le(rxBuffer + frameLength - CRC_SIZE);
    uint16_t calculatedCrc =
        crc16CcittFalse(rxBuffer + sizeof(MAGIC),
                        frameLength - sizeof(MAGIC) - CRC_SIZE);
    // CRC 错误时丢弃一个字节，再次搜索帧头，防止后续帧被连带丢失。
    if (receivedCrc != calculatedCrc) {
      discardPrefix(1);
      ++crcErrors;
      continue;
    }

    // 只有通过协议和 CRC 校验的帧才允许发往 PC。
    ++validFrames;
    if (client && client.connected()) {
      if (!sendAll(rxBuffer, frameLength)) {
        client.stop();
        ++droppedFrames;
      }
    } else {
      ++droppedFrames;
    }
    discardPrefix(frameLength);
  }
}

// 批量读取 UART2 数据，并在每次读取后立即尝试拆帧。
static void readUart() {
  while (Serial2.available() > 0) {
    size_t freeSpace = RX_BUFFER_SIZE - rxLength;
    // 极端情况下缓存已满，丢弃最旧的一个字节并重新同步。
    if (freeSpace == 0) {
      discardPrefix(1);
      ++droppedFrames;
      freeSpace = 1;
    }
    size_t availableBytes = (size_t)Serial2.available();
    size_t toRead = availableBytes < freeSpace ? availableBytes : freeSpace;
    rxLength += Serial2.readBytes(rxBuffer + rxLength, toRead);
    processFrames();
  }
}

// PC 下发的 STOP、CAPTURE 等短命令直接转发到 STM32 UART2_TX。
// 图像和控制使用 TCP 全双工的两个方向，不会混入 STM32 上行二进制帧。
static void forwardPcCommands() {
  uint8_t commandBuffer[64];
  while (client && client.connected() && client.available() > 0) {
    size_t availableBytes = (size_t)client.available();
    size_t toRead = availableBytes < sizeof(commandBuffer)
                        ? availableBytes : sizeof(commandBuffer);
    int received = client.read(commandBuffer, toRead);
    if (received <= 0) return;
    size_t written = Serial2.write(commandBuffer, (size_t)received);
    controlBytes += written;
  }
}

// 维护到 PC 的 TCP 连接；断开后每 2 秒主动重连一次。
static void maintainTcpConnection() {
  if (WiFi.status() != WL_CONNECTED || client.connected()) return;

  uint32_t now = millis();
  if (now - lastTcpRetryMs < 2000) return;
  lastTcpRetryMs = now;

  client.stop();
  Serial.print("Connecting to PC: ");
  Serial.print(PC_IP);
  Serial.print(":");
  Serial.println(TCP_PORT);

  if (client.connect(PC_IP, TCP_PORT)) {
    client.setNoDelay(true);
    Serial.println("PC TCP connected");
  } else {
    Serial.println("PC TCP connection failed");
  }
}

// 维护 STA 网络连接，并在连接状态变化时输出新的局域网 IP。
static void maintainWiFi() {
  static bool wasConnected = false;
  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!wasConnected) {
      Serial.println("Wi-Fi connected");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("PC target: ");
      Serial.print(PC_IP);
      Serial.print(":");
      Serial.println(TCP_PORT);
    }
    wasConnected = true;
    return;
  }

  if (wasConnected) {
    Serial.println("Wi-Fi disconnected");
    if (client) client.stop();
  }
  wasConnected = false;

  // 自动重连失败时，每 5 秒主动发起一次重连。
  uint32_t now = millis();
  if (now - lastWiFiRetryMs >= 5000) {
    lastWiFiRetryMs = now;
    Serial.println("Retrying Wi-Fi...");
    WiFi.reconnect();
  }
}

void setup() {
  // USB 调试串口，串口监视器波特率为 115200。
  Serial.begin(115200);

  // 在 begin() 前扩大 UART2 接收缓存，降低高速串口溢出概率。
  Serial2.setRxBufferSize(16384);
  Serial2.setTimeout(2);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // STA 模式：ESP32-S3 与 PC 连接同一个路由器。
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  maintainWiFi();

  Serial.println("MLX90640 + OV2640 bridge ready");
}

void loop() {
  // 先维护 Wi-Fi 和到 PC 的 TCP 连接，再持续接收 STM32 图像帧。
  maintainWiFi();
  maintainTcpConnection();
  forwardPcCommands();
  readUart();

  // 每 2 秒输出一次统计信息，避免逐字节打印拖慢 UART。
  uint32_t now = millis();
  if (now - lastStatsMs >= 2000) {
    lastStatsMs = now;
    Serial.printf("wifi=%s frames=%lu crc_errors=%lu dropped=%lu buffered=%u controls=%lu client=%s\n",
                  WiFi.status() == WL_CONNECTED ? "yes" : "no",
                  (unsigned long)validFrames, (unsigned long)crcErrors,
                  (unsigned long)droppedFrames, (unsigned)rxLength,
                  (unsigned long)controlBytes,
                  (client && client.connected()) ? "yes" : "no");
  }
  delay(1);
}
