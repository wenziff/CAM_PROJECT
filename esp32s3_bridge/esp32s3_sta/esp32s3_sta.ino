#include <WiFi.h>

const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WiFiServer server(8888);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(2000);  // ⚠️ 关键：等待2秒，让电脑端的串口驱动完全准备好
  Serial.println("Setup Start"); // 加这一行来验证程序是否跑起来了
  Serial2.begin(115200, SERIAL_8N1, 18, 17); 

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // ⚠️ 这个IP地址记下来，给上位机填！

  // 🔥 关键：关闭WiFi节能模式，提升图像传输速率和稳定性
  WiFi.setSleep(false); 

  server.begin();
  Serial.println("TCP Server started on port 8888");
}

void loop() {
  // 检查新客户端（逻辑保留你原来的风格）
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      Serial.println("New Image Client connected");
    }
    delay(10);
    return;
  }

  // 从Serial2（接STM32）接收图像数据并转发到TCP
  if (Serial2.available()) {
    // 为了图像传输，尽量用批量读取，减少循环开销
    while (Serial2.available()) {
      uint8_t c = Serial2.read();
      if (client && client.connected()) {
        client.write(c);  // 发送图像字节
      }
    }
    // 图像数据量大，不要每发一个字节都flush，等一帧发完再flush
    // client.flush(); // 可在接收完一帧图像后调用一次
  }

  // TCP -> Serial2（可选）
  while (client.available()) {
    Serial2.write(client.read());
  }
}
