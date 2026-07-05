#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_pm.h>
#include <soc/rtc.h>

// WiFi 信息
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// PC 端 TCP 服务器信息（请修改为 PC 的实际局域网 IP）
const char* PC_IP = "10.211.44.196";
const uint16_t PC_PORT = 8888;

WiFiClient tcpClient;   // TCP 客户端对象

#if(1)
void setup() {
  setCpuFrequencyMhz(80);
  Serial2.begin(115200, SERIAL_8N1, 17, 18);  // RX=GPIO17, TX=GPIO18
  Serial.println("📡 串口2 已启动 (RX=17, TX=18)");
  Serial.begin(115200);
  delay(1000);  // 等待串口稳定
  

  Serial.println("\n========== ESP32-S3 主动连接 PC ==========");
  Serial.print("正在连接 WiFi: ");
  Serial.println(ssid);

  // 设置为 Station 模式
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("mode set successful");

  // 等待 WiFi 连接（最多尝试 20 秒）
  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED && tryCount < 20) {
    delay(1000);
    Serial.print(".");
    tryCount++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi 连接成功！");
    Serial.print("📡 IP 地址: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 信号强度: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    // 连接 PC 端的 TCP 服务器
    Serial.print("正在连接 PC (");
    Serial.print(PC_IP);
    Serial.print(":");
    Serial.print(PC_PORT);
    Serial.println(")...");

    if (tcpClient.connect(PC_IP, PC_PORT)) {
      Serial.println("✅ TCP 连接成功！");
      // 发送消息
      tcpClient.println("Hello ESP32");
      Serial.println("📤 已发送: Hello ESP32");
    } else {
      Serial.println("❌ TCP 连接失败！请检查：");
      Serial.println("  1. PC 是否已启动 TCP 服务器");
      Serial.println("  2. PC 防火墙是否放行了端口");
      Serial.println("  3. PC 的 IP 和端口是否正确");
    }
  } else {
    Serial.println("❌ WiFi 连接失败！请检查：");
    Serial.println("  1. WiFi 名称/密码是否正确");
    Serial.println("  2. 路由器是否开启了 2.4G 频段");
    Serial.println("  3. 天线是否接好");
  }
}

void loop() {
  // 如果 TCP 断开，可在此处加重连逻辑（按需）
  if (!tcpClient.connected()) {
    // 重连代码可以后补
  } else {
    // 从 STM32 的串口读取数据并转发
    while (Serial2.available()) {
      char c = Serial2.read();
      tcpClient.write(c);   // 逐字节发送，保持原始数据
      Serial.write(c);      // 同时回显到电脑，方便观察
    }
  }
  delay(1);  // 避免看门狗复位
}
#endif
