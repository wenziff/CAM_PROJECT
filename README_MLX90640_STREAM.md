# MLX90640 STM32 → ESP32-S3 → PC 热成像链路

本实现对应参考工程中的 **MLX90640**（32×24 热电堆阵列）。如果“MLX9640”不是笔误，请先确认传感器完整型号；当前驱动、寄存器和标定算法均针对 MLX90640。

## 1. 接线

| 连接 | 引脚 |
|---|---|
| H16.1 / H16.2 | MLX90640 3.3 V / GND |
| H16.3（MLX_SCL） | STM32 PA8（U9.67，I2C3_SCL） |
| H16.4（MLX_SDA） | STM32 PC9（U9.66，I2C3_SDA） |
| H22.3（PA9） | STM32 USART1_TX → ESP32-S3 GPIO17（RX） |
| H22.4（PA10） | STM32 USART1_RX ← ESP32-S3 GPIO18（TX，可选） |
| H22.2 | STM32 GND ↔ ESP32 GND |

网表确认 OV_D3 位于 PE1（U9.98），所以 PC9 与并口摄像头没有引脚冲突；同时修正了 OV_D5=PD3。当前热成像主循环仍不启动 DCMI。

网表中没有看到 MLX_SCL/MLX_SDA 的板载上拉电阻。若 H16 接的是不带上拉的裸传感器/模块，SDA、SCL 必须各加 2.2–4.7 kΩ 上拉至 3.3 V；若模块上没有上拉电阻，建议各加 2.2–4.7 kΩ。ESP32 的 TX/GPIO18 当前不参与图像上行，可不接。

## 2. 固件配置

STM32：

- I2C3：400 kHz，7-bit；MLX90640 地址 0x33
- MLX90640：18-bit ADC、Chess mode、8 Hz
- USART1：921600，8N1
- 像素：摄氏温度 ×100，little-endian int16
- 打开并编译 [MDK-ARM/CAM1.0.uvprojx](MDK-ARM/CAM1.0.uvprojx)

ESP32-S3：

- Arduino 草图：[esp32s3_bridge/esp32s3_MLX90640/esp32s3_MLX90640.ino](esp32s3_bridge/esp32s3_MLX90640/esp32s3_MLX90640.ino)
- Wi-Fi 模式：STA；ESP32-S3 与 PC 连接同一个路由器
- 路由器：good
- ESP32-S3 作为 TCP 客户端，主动连接 PC 10.211.44.196:8888
- PC 上位机作为 TCP 服务端，默认监听 0.0.0.0:8888
- UART2：RX=GPIO17，921600 baud
- 桥接端先校验完整帧和 CRC，再批量转发，不逐字节打印

## 3. PC 端

安装依赖：

    cd pc_receiver
    python -m pip install -r requirements.txt

先让 PC 与 ESP32-S3 连接同一个路由器，并确认 PC 的地址为 10.211.44.196。先启动 PC 接收服务，再给 ESP32-S3 上电：

Windows 防火墙需要允许 Python 接收入站 TCP 8888。10.211.44.196 必须是 ESP32 能访问的局域网地址；若它属于虚拟机/NAT 虚拟网卡，请改用桥接网络或填写 PC 的实际 Wi-Fi 地址。

    python mlx90640_viewer.py

如需只监听指定的 PC 网卡地址：

    python mlx90640_viewer.py --bind 10.211.44.196

无界面诊断：

    python mlx90640_viewer.py --headless

协议自检（无需硬件）：

    python mlx90640_viewer.py --self-test

可用 --flip-x 水平镜像；用 --save-dir frames 每 8 帧保存一份温度矩阵，间隔可由 --save-every 修改。

## 4. 帧协议（little-endian）

| 偏移 | 长度 | 含义 |
|---:|---:|---|
| 0 | 4 | 魔数 ASCII MLX4 |
| 4 | 1 | 协议版本，当前为 1 |
| 5 | 1 | 类型：0x01 温度，0x7F 状态 |
| 6 | 1 | 宽度（温度帧为 32） |
| 7 | 1 | 高度（温度帧为 24） |
| 8 | 4 | 帧序号 |
| 12 | 4 | STM32 启动后的毫秒数 |
| 16 | 2 | payload 长度 |
| 18 | 2 | 最低温，0.01°C |
| 20 | 2 | 最高温，0.01°C |
| 22 | N | payload；温度帧为 1536 字节 |
| 22+N | 2 | CRC16/CCITT-FALSE，覆盖偏移 4 到 payload 末尾 |

完整温度帧为 1560 字节。状态帧 payload 为 little-endian int32 错误码。

## 5. OV2640 实时 LCD 显示

- OV2640 通过 I2C2（PB10/PB11）配置为 QVGA RGB565，图像由 DCMI + DMA1 采集。
- LCD 参数与 `D:\lww\yanjiushengdiansai\4.0` 参考工程保持一致，有效区域为 240×240。
- 摄像头输出为 320×240，DCMI 居中裁剪为 240×240，并铺满 LCD，不再保留越界边带。
- 上电后先显示 1 秒红、绿、蓝、白彩条：彩条正常可确认 LCD 与 SPI 通道正常，随后再进入实时相机显示。

## 6. KEY1 拍照并发送到 PC

- KEY1 连接 PA0，按下时接地；GPIO 使用内部上拉，按下沿消抖后只拍摄一次。
- STM32 锁定当前 240×240 RGB565 帧，共 115200 字节，并拆成 29 个分片。
- 每个分片沿用 `MLX4` 二进制帧头，帧类型为 `0x02`，数据不超过 4096 字节并带 CRC16。
- ESP32-S3 校验每个分片后，通过 STA 模式下的 TCP 8888 转发到 PC。
- PC 程序自动重组照片，保存到 `pc_receiver/captures/ov2640_XXXXXXXXXX.bmp`，图形界面右侧同步显示最新照片。
- KEY2（PC1，按下接地）用于保存热力图：按下一次后，下一张完整热图以快照帧发送，PC 将 PNG 和原始温度 NPY 保存到 `pc_receiver/thermal/`。
- 默认不自动保存实时热图；如需额外定时保存，可用 `--save-every N`，其中 `--save-every 1` 表示逐帧保存。
- 拍照传输在 921600 波特率下约占用 1.3 秒；传输期间 LCD 保持当前照片，完成后恢复实时画面。

PC 端启动命令：

```powershell
python pc_receiver\mlx90640_viewer.py
```
- 帧缓冲使用 AXI SRAM 预留区 `0x24040000`；采集完成后再通过 SPI1 整帧刷屏，避免画面撕裂。
- MLX90640 改为非阻塞子页轮询，因此 OV2640 本地显示与 STM32→ESP32→PC 热成像传输可以同时运行。

## 7. PC 火情监控界面与停车控制

- 图形界面同时显示可见光、红外热力图和双光融合图，三路图像可用界面复选按钮分别隐藏或显示。
- 热力图按传感器安装方向统一逆时针旋转 90°；火情位置和融合图使用旋转后的坐标。
- 界面顶部包含火情状态、火情位置和 LED 指示灯；检测到火情时 LED 变红并循环播放 `pc_receiver/the_sound_of_fire_alarm.mp3`，可用“警报消音”按钮停止声音。
- PC 通过 TCP 向 ESP32-S3 发送 `STOP` 和 `CAPTURE`，ESP32-S3 经 UART2_TX 转发给 STM32；STM32 停止双路电机 PWM，并上传当前可见光帧。
- 火情发生后界面锁定触发时刻的热图，并在当前可见光抓拍到达后生成融合图。
- 当前仓库没有训练模型及权重，`FireDetector.detect()` 暂用最高温阈值作为可运行的联调检测器。默认阈值为 60 ℃，可用 `--fire-threshold` 修改；接入真实双光模型时替换该方法即可。

示例：

```powershell
python pc_receiver\mlx90640_viewer.py --fire-threshold 60
```

如只调试画面而不播放声音，可增加 `--no-alarm`。
也可通过 `--alarm-sound 其他音效.mp3` 临时指定另一段警报音效。
