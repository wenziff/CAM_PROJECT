// ESP32S3 UART↔TCP 透传桥

#include <WiFi.h>

const char* ssid     = "ESP32S3-CAM";
const char* password = "CHANGE_ME_1234";

WiFiServer server(8888);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 18, 17);  // RX1=GPIO18, TX1=GPIO17

  WiFi.softAP(ssid, password,1);
  server.begin();
}

void loop() {
  // 1. 检查并接受新客户端（仅当当前客户端无效时）
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();   // 使用临时变量
    if (newClient) {
      client = newClient;
      Serial.print("New client connected: ");
      Serial.println(client.remoteIP());
      // 可选：发送欢迎消息
      client.println("Hello from ESP32");
      client.flush();
    }
    delay(10);
    return;  // 避免后续处理无效client
  }

  // 2. 现在 client 一定有效，转发数据
  // 从 STM32 接收并转发到 TCP
  if (Serial2.available()) {
    Serial.println("Serial2 data available!");   // 只要进入 if 就打印
    while (Serial2.available()) {
      uint8_t c = Serial2.read();
      Serial.print("Received byte: 0x");
      Serial.println(c, HEX);  // 打印十六进制值
      // 转发到 TCP
      if (client && client.connected()) {
        size_t sent = client.write(c);
        if (sent != 1) {
          Serial.println("TCP write failed!");
        }
      } else {
        Serial.println("Client not connected, dropping byte");
      }
    }
    client.flush();
  }

  // 3. 可选：TCP -> STM32 转发（如果不需要可暂时屏蔽）
  while (client.available()) {
    uint8_t c = client.read();
    Serial2.write(c);
  }

  delay(1);  // 或 yield()
}
