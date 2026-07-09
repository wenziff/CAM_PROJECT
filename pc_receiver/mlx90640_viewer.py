#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""接收并显示 STM32H7 识别火情后发送的可见光、热图和融合图。"""
from __future__ import annotations

import argparse
import dataclasses
import socket
import struct
import threading
import time
import tempfile
import collections.abc
from pathlib import Path
import numpy as np

# STM32、ESP32-S3、PC 三端共用的二进制协议参数。
# 温度帧：22 字节帧头 + 1536 字节像素数据 + 2 字节 CRC。
MAGIC = b"MLX4"
VERSION = 1
HEADER_SIZE = 22
CRC_SIZE = 2
THERMAL_PAYLOAD_SIZE = 32 * 24 * 2
MAX_PAYLOAD_SIZE = 4096
PHOTO_WIDTH = 240
PHOTO_HEIGHT = 240
PHOTO_RGB565_SIZE = PHOTO_WIDTH * PHOTO_HEIGHT * 2
# 0x01 是实时温度图，0x02 是照片分片，0x03 是 STM32 火情热图快照。
# PC 以 0x03 作为“STM32 已识别到 fire”的事件标志，不再进行二次识别。
TEMPERATURE_FRAME = 0x01
PHOTO_CHUNK_FRAME = 0x02
THERMAL_SNAPSHOT_FRAME = 0x03
STATUS_FRAME = 0x7F
# int16 最小值专门代表无效温度，不参与热力图计算。
INVALID_TEMPERATURE = -32768
# STM32 自定义错误码说明；其他负数通常来自 MLX90640 官方 API。
STATUS_TEXT = {
    -100: "MLX90640 not found on I2C3",
    -101: "MLX90640 frame timeout",
    -102: "STM32 UART transmit failed",
    -103: "STM32 photo argument invalid",
}


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    """计算 CRC16/CCITT-FALSE，参数必须与 STM32、ESP32 保持一致。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclasses.dataclass(slots=True)
class Frame:
    """保存一帧已经通过长度和 CRC 校验的数据。"""

    frame_type: int
    width: int
    height: int
    sequence: int
    timestamp_ms: int
    minimum_centi_c: int
    maximum_centi_c: int
    payload: bytes
    received_at: float


class FrameParser:
    """从无消息边界的 TCP 字节流中搜索并提取完整协议帧。"""

    def __init__(self) -> None:
        # 一次 recv() 可能收到半帧、一帧或连续多帧，因此需要累计缓存。
        self.buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0

    def reset(self) -> None:
        """新连接建立时清空上一次连接遗留的残帧。"""
        self.buffer.clear()

    def feed(self, data: bytes) -> collections.abc.Iterable[Frame]:
        """加入新数据，并返回本次能够解析出的所有完整帧。"""
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            # 搜索 ASCII 魔数 MLX4，自动跳过串口启动噪声或损坏字节。
            position = self.buffer.find(MAGIC)
            if position < 0:
                # 保留最后 3 字节，防止魔数横跨两次 TCP 接收。
                if len(self.buffer) > len(MAGIC) - 1:
                    del self.buffer[: -(len(MAGIC) - 1)]
                break
            if position:
                del self.buffer[:position]
            if len(self.buffer) < HEADER_SIZE:
                break

            version, frame_type, width, height = self.buffer[4:8]
            if version != VERSION:
                # 版本错误时只向后移动一字节，继续寻找下一个有效帧头。
                del self.buffer[0]
                self.format_errors += 1
                continue

            sequence, timestamp_ms, payload_length, minimum, maximum = struct.unpack_from(
                "<IIHhh", self.buffer, 8
            )
            # 防止损坏的长度字段让接收缓存无限等待。
            if payload_length > MAX_PAYLOAD_SIZE:
                del self.buffer[0]
                self.format_errors += 1
                continue

            frame_length = HEADER_SIZE + payload_length + CRC_SIZE
            if len(self.buffer) < frame_length:
                # 当前数据只是残帧，等待下一次 recv() 补齐。
                break

            packet = bytes(self.buffer[:frame_length])
            received_crc = struct.unpack_from("<H", packet, frame_length - CRC_SIZE)[0]
            calculated_crc = crc16_ccitt_false(packet[4:-CRC_SIZE])
            if received_crc != calculated_crc:
                # CRC 错误时丢弃一个字节重新同步，尽量保留后续正常帧。
                del self.buffer[0]
                self.crc_errors += 1
                continue

            frames.append(Frame(
                frame_type=frame_type, width=width, height=height,
                sequence=sequence, timestamp_ms=timestamp_ms,
                minimum_centi_c=minimum, maximum_centi_c=maximum,
                payload=packet[HEADER_SIZE:-CRC_SIZE],
                received_at=time.monotonic(),
            ))
            del self.buffer[:frame_length]
        return frames


class Receiver(threading.Thread):
    """PC 端 TCP 服务线程，等待 ESP32-S3 主动连接并接收图像。"""

    def __init__(self, bind_address: str, port: int, photo_dir: Path,
                 thermal_dir: Path, thermal_flip_x: bool) -> None:
        super().__init__(daemon=True)
        self.bind_address = bind_address
        self.port = port
        self.photo_dir = photo_dir
        self.thermal_dir = thermal_dir
        self.thermal_flip_x = thermal_flip_x
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest: Frame | None = None
        self.latest_fire_snapshot: Frame | None = None
        self.fire_event_count = 0
        self.connection_state = "starting"
        self.parser = FrameParser()
        self.frames_received = 0
        self.sequence_gaps = 0
        self.last_sequence: int | None = None
        self.latest_photo: np.ndarray | None = None
        self.latest_photo_sequence: int | None = None
        self.photos_received = 0           # 本地照片保存计数（用作文件名序号）
        self.thermal_snapshots_saved = 0   # 本地热图快照保存计数（用作文件名序号）
        self.photo_sequence: int | None = None
        self.photo_width = 0
        self.photo_height = 0
        self.photo_chunk_count = 0
        self.photo_chunks: dict[int, bytes] = {}

    def stop(self) -> None:
        """通知后台接收线程退出。"""
        self.stop_event.set()

    def snapshot(self) -> tuple[Frame | None, str]:
        """线程安全地获取最新温度帧和当前网络状态。"""
        with self.lock:
            return self.latest, self.connection_state

    def photo_snapshot(self) -> tuple[np.ndarray | None, int | None]:
        """线程安全地获取最近一次完整的 OV2640 照片。"""
        with self.lock:
            return self.latest_photo, self.latest_photo_sequence

    def fire_snapshot(self) -> tuple[Frame | None, int]:
        """获取最近一次 STM32 火情快照及单调递增的事件编号。"""
        with self.lock:
            return self.latest_fire_snapshot, self.fire_event_count

    def _set_state(self, state: str) -> None:
        """更新供主线程或图形界面读取的连接状态。"""
        with self.lock:
            self.connection_state = state

    def _handle_frame(self, frame: Frame) -> None:
        """处理状态帧或温度帧，并统计帧序号是否连续。"""
        if frame.frame_type == PHOTO_CHUNK_FRAME:
            self._handle_photo_chunk(frame)
            return
        if frame.frame_type == STATUS_FRAME and len(frame.payload) == 4:
            code = struct.unpack("<i", frame.payload)[0]
            print(f"STM32 status {code}: {STATUS_TEXT.get(code, 'MLX90640 API/I2C error')}")
            return
        if (frame.frame_type not in (TEMPERATURE_FRAME,
                                     THERMAL_SNAPSHOT_FRAME)
                or frame.width != 32
                or frame.height != 24
                or len(frame.payload) != THERMAL_PAYLOAD_SIZE):
            return

        if self.last_sequence is not None:
            # 序号不连续说明 UART、Wi-Fi 或 PC 接收过程中发生过丢帧。
            expected = (self.last_sequence + 1) & 0xFFFFFFFF
            if frame.sequence != expected:
                self.sequence_gaps += (frame.sequence - expected) & 0xFFFFFFFF
        self.last_sequence = frame.sequence
        self.frames_received += 1
        with self.lock:
            self.latest = frame
            if frame.frame_type == THERMAL_SNAPSHOT_FRAME:
                self.latest_fire_snapshot = frame
                self.fire_event_count += 1
        if frame.frame_type == THERMAL_SNAPSHOT_FRAME:
            self.thermal_snapshots_saved += 1
            local_index = self.thermal_snapshots_saved
            image = temperatures_from_frame(frame, self.thermal_flip_x)
            save_frame(self.thermal_dir, image, index=local_index)
            print(
                f"STM32 fire thermal snapshot saved: "
                f"{self.thermal_dir / f'mlx90640_{local_index:06d}.png'}"
            )

    def _handle_photo_chunk(self, frame: Frame) -> None:
        """按照片序号和分片序号重组一张 RGB565 照片。"""
        chunk_index = frame.minimum_centi_c
        chunk_count = frame.maximum_centi_c
        if (frame.width != PHOTO_WIDTH or frame.height != PHOTO_HEIGHT
                or chunk_count <= 0 or chunk_count > 256
                or chunk_index < 0 or chunk_index >= chunk_count):
            return

        if (frame.sequence != self.photo_sequence
                or chunk_count != self.photo_chunk_count):
            self.photo_sequence = frame.sequence
            self.photo_width = frame.width
            self.photo_height = frame.height
            self.photo_chunk_count = chunk_count
            self.photo_chunks = {}

        self.photo_chunks[chunk_index] = frame.payload
        if len(self.photo_chunks) != self.photo_chunk_count:
            return

        try:
            payload = b"".join(
                self.photo_chunks[index]
                for index in range(self.photo_chunk_count)
            )
        except KeyError:
            return
        if len(payload) != self.photo_width * self.photo_height * 2:
            self.photo_chunks = {}
            return

        image = rgb565_to_rgb(payload, self.photo_width, self.photo_height)
        self.photos_received += 1
        local_index = self.photos_received
        path = save_photo_bmp(self.photo_dir, local_index, image)
        with self.lock:
            self.latest_photo = image
            self.latest_photo_sequence = local_index
        print(f"OV2640 photo saved: {path}")
        self.photo_chunks = {}

    def run(self) -> None:
        """监听 TCP 端口；ESP32 断线后继续等待下一次连接。"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                # 允许程序重启后立即重新绑定端口，减小 TIME_WAIT 的影响。
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.bind_address, self.port))
                # 允许 ESP32 复位后的新连接先进入等待队列。
                server.listen(4)
                server.settimeout(1.0)
                self._set_state(f"listening on {self.bind_address}:{self.port}")
                print(f"Waiting for ESP32 on {self.bind_address}:{self.port}")

                while not self.stop_event.is_set():
                    try:
                        connection, address = server.accept()
                    except TimeoutError:
                        continue

                    with connection:
                        # 定时退出 recv()，让线程能够及时响应关闭请求。
                        connection.settimeout(1.0)
                        self.parser.reset()
                        last_data_time = time.monotonic()
                        self._set_state(f"connected: {address[0]}")
                        print(f"ESP32 connected from {address[0]}:{address[1]}")
                        while not self.stop_event.is_set():
                            try:
                                chunk = connection.recv(8192)
                            except TimeoutError:
                                # ESP32 硬复位时旧套接字可能收不到正常 FIN；
                                # 热图正常时持续有数据，3 秒静默即可判定为失效连接。
                                if time.monotonic() - last_data_time > 3.0:
                                    print("ESP32 connection inactive; accepting a new connection")
                                    break
                                continue
                            except OSError:
                                break
                            if not chunk:
                                break
                            last_data_time = time.monotonic()
                            for frame in self.parser.feed(chunk):
                                self._handle_frame(frame)
                        self._set_state(f"listening on {self.bind_address}:{self.port}")
                        print("ESP32 disconnected; waiting for reconnection")
        except OSError as error:
            self._set_state(f"TCP server error: {error}")
            print(f"TCP server error: {error}")


def temperatures_from_frame(frame: Frame, flip_x: bool) -> np.ndarray:
    """转换温度矩阵，并按传感器安装方向逆时针旋转 90°。"""
    raw = np.frombuffer(frame.payload, dtype="<i2").astype(np.float32)
    raw[raw == INVALID_TEMPERATURE] = np.nan
    image = raw.reshape((frame.height, frame.width)) / 100.0
    if flip_x:
        image = np.fliplr(image)
    return np.rot90(image, 1)


def describe_fire_location(row: int, column: int,
                           height: int, width: int) -> str:
    """把热图像素坐标转换为便于现场查看的九宫格位置。"""
    horizontal = ("左侧" if column < width / 3 else
                  "右侧" if column >= width * 2 / 3 else "中央")
    vertical = ("上方" if row < height / 3 else
                "下方" if row >= height * 2 / 3 else "中部")
    if horizontal == "中央" and vertical == "中部":
        area = "画面中央"
    else:
        area = vertical + horizontal
    return f"{area}（x={column}, y={row}）"


def resize_nearest(image: np.ndarray, height: int, width: int) -> np.ndarray:
    """只依赖 NumPy 的最近邻缩放，用于把 32×24 热图映射到照片尺寸。"""
    source_height, source_width = image.shape[:2]
    y_indices = np.linspace(0, source_height - 1, height).astype(np.intp)
    x_indices = np.linspace(0, source_width - 1, width).astype(np.intp)
    return image[y_indices[:, None], x_indices[None, :]]


def build_fusion_image(visible: np.ndarray | None,
                       thermal: np.ndarray) -> np.ndarray:
    """生成可见光与伪彩热图的透明叠加图。"""
    from matplotlib import colormaps

    if visible is None:
        visible_image = np.zeros((PHOTO_HEIGHT, PHOTO_WIDTH, 3), dtype=np.uint8)
    else:
        visible_image = visible.astype(np.uint8, copy=False)
    resized = resize_nearest(
        thermal, visible_image.shape[0], visible_image.shape[1]
    )
    valid = resized[np.isfinite(resized)]
    if valid.size == 0:
        return visible_image.copy()
    low = float(np.percentile(valid, 5.0))
    high = float(np.percentile(valid, 99.0))
    if high - low < 1.0:
        high = low + 1.0
    normalized = np.nan_to_num((resized - low) / (high - low), nan=0.0)
    thermal_rgb = colormaps["inferno"](
        np.clip(normalized, 0.0, 1.0)
    )[:, :, :3]
    fused = 0.58 * (visible_image.astype(np.float32) / 255.0) + 0.42 * thermal_rgb
    return np.clip(fused * 255.0, 0, 255).astype(np.uint8)


class AlarmController:
    """使用 Windows MCI 循环播放 MP3，避免阻塞 Matplotlib 界面线程。"""

    def __init__(self, sound_path: Path, enabled: bool = True) -> None:
        self.sound_path = sound_path
        self.enabled = enabled
        self.active = threading.Event()
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def start(self) -> None:
        if self.enabled:
            self.active.set()

    def silence(self) -> None:
        self.active.clear()

    def close(self) -> None:
        self.active.clear()
        self.closed.set()
        self.worker.join(timeout=1.0)

    def _run(self) -> None:
        if self.enabled and self.sound_path.is_file():
            try:
                import ctypes
                mci = ctypes.windll.winmm.mciSendStringW
                alias = f"fire_alarm_{id(self)}"
                path = str(self.sound_path.resolve()).replace('"', '')
                if mci(f'open "{path}" type mpegvideo alias {alias}',
                       None, 0, None) == 0:
                    self._run_mci(mci, alias)
                    return
                print(f"Unable to open alarm MP3: {self.sound_path}")
            except (AttributeError, OSError) as error:
                print(f"Unable to use Windows MP3 alarm: {error}")
        self._run_fallback_beep()

    def _run_mci(self, mci, alias: str) -> None:
        playing = False
        try:
            while not self.closed.is_set():
                requested = self.active.is_set()
                if requested and not playing:
                    mci(f"seek {alias} to start", None, 0, None)
                    playing = mci(f"play {alias} repeat", None, 0, None) == 0
                elif not requested and playing:
                    mci(f"stop {alias}", None, 0, None)
                    playing = False
                self.closed.wait(0.1)
        finally:
            mci(f"stop {alias}", None, 0, None)
            mci(f"close {alias}", None, 0, None)

    def _run_fallback_beep(self) -> None:
        try:
            import winsound
        except ImportError:
            winsound = None
        while not self.closed.is_set():
            if not self.active.wait(0.1):
                continue
            if winsound is not None:
                try:
                    winsound.Beep(1800, 260)
                except RuntimeError:
                    self.closed.wait(0.3)
            else:
                print("\a", end="", flush=True)
                self.closed.wait(0.5)


def rgb565_to_rgb(payload: bytes, width: int, height: int) -> np.ndarray:
    """将 OV2640 高字节优先的 RGB565 数据转换为 RGB888 图像。"""
    pixels = np.frombuffer(payload, dtype=">u2").reshape((height, width))
    red = ((pixels >> 11) & 0x1F).astype(np.uint16) * 255 // 31
    green = ((pixels >> 5) & 0x3F).astype(np.uint16) * 255 // 63
    blue = (pixels & 0x1F).astype(np.uint16) * 255 // 31
    return np.stack((red, green, blue), axis=-1).astype(np.uint8)


def save_photo_bmp(directory: Path, sequence: int,
                   image: np.ndarray) -> Path:
    """不依赖 Pillow，将 RGB888 数组保存为 Windows 24 位 BMP。
       sequence 现在代表本地保存序号，生成 ov2640_000001.bmp 格式的文件名。
    """
    directory.mkdir(parents=True, exist_ok=True)
    height, width, _ = image.shape
    row_bytes = width * 3
    padding = b"\x00" * ((4 - row_bytes % 4) % 4)
    bgr = image[:, :, ::-1]
    pixel_data = b"".join(
        bgr[row].tobytes() + padding
        for row in range(height - 1, -1, -1)
    )
    pixel_offset = 14 + 40
    file_size = pixel_offset + len(pixel_data)
    file_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0,
                              pixel_offset)
    dib_header = struct.pack(
        "<IIIHHIIIIII", 40, width, height, 1, 24, 0,
        len(pixel_data), 2835, 2835, 0, 0
    )
    path = directory / f"ov2640_{sequence:06d}.bmp"
    path.write_bytes(file_header + dib_header + pixel_data)
    return path


def save_frame(directory: Path, image: np.ndarray, *,
               index: int | None = None, frame: Frame | None = None) -> None:
    """同时保存彩色热力图 PNG 和可继续分析的温度矩阵 NPY。

    当 index 不为 None 时使用本地序号（6 位零填充），
    否则使用帧序号（10 位零填充），适用于自动保存场景。
    """
    from matplotlib import image as matplotlib_image

    directory.mkdir(parents=True, exist_ok=True)
    if index is not None:
        stem = f"mlx90640_{index:06d}"
    elif frame is not None:
        stem = f"mlx90640_{frame.sequence:010d}"
    else:
        raise ValueError("Either index or frame must be provided")

    np.save(directory / f"{stem}.npy", image)

    valid = image[np.isfinite(image)]
    if valid.size == 0:
        return
    minimum = float(np.min(valid))
    maximum = float(np.max(valid))
    if maximum - minimum < 1.0:
        minimum -= 0.5
        maximum += 0.5
    matplotlib_image.imsave(
        directory / f"{stem}.png", image,
        cmap="inferno", vmin=minimum, vmax=maximum,
    )


def run_headless(receiver: Receiver, args: argparse.Namespace) -> None:
    """无界面运行，每秒打印温度、帧数、丢帧和 CRC 统计。"""
    last_sequence = None
    last_photo_sequence = None
    last_report = 0.0
    image = np.full((24, 32), np.nan)
    try:
        while True:
            frame, state = receiver.snapshot()
            if frame is not None and frame.sequence != last_sequence:
                image = temperatures_from_frame(frame, args.flip_x)
                last_sequence = frame.sequence
                if (args.save_every > 0
                        and frame.frame_type == TEMPERATURE_FRAME
                        and frame.sequence % args.save_every == 0):
                    save_frame(args.save_dir, image, frame=frame)
            _photo, photo_sequence = receiver.photo_snapshot()
            if photo_sequence is not None and photo_sequence != last_photo_sequence:
                last_photo_sequence = photo_sequence
                print(f"received OV2640 photo sequence={photo_sequence}")
            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                if frame is None:
                    print(state)
                else:
                    print(
                        f"seq={frame.sequence} min={np.nanmin(image):.2f}C "
                        f"max={np.nanmax(image):.2f}C frames={receiver.frames_received} "
                        f"gaps={receiver.sequence_gaps} crc={receiver.parser.crc_errors}"
                    )
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass


def run_gui(receiver: Receiver, args: argparse.Namespace) -> None:
    """显示三路图像；火情判定和电机控制全部由 STM32H7 完成。"""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle
    from matplotlib.widgets import Button, CheckButtons

    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei", "SimHei", "DejaVu Sans"
    ]
    plt.rcParams["axes.unicode_minus"] = False
    alarm = AlarmController(args.alarm_sound, enabled=not args.no_alarm)

    figure, (photo_axis, thermal_axis, fusion_axis) = plt.subplots(
        1, 3, figsize=(17, 7)
    )
    figure.subplots_adjust(left=0.04, right=0.98, bottom=0.23, top=0.78,
                           wspace=0.22)
    try:
        figure.canvas.manager.set_window_title("家庭自主式火源巡检系统")
    except AttributeError:
        pass

    blank_photo = np.zeros((PHOTO_HEIGHT, PHOTO_WIDTH, 3), dtype=np.uint8)
    photo_plot = photo_axis.imshow(blank_photo, interpolation="nearest")
    photo_axis.set_title("可见光图像（等待采集）")
    photo_axis.set_axis_off()
    thermal_plot = thermal_axis.imshow(
        np.zeros((32, 24), dtype=np.float32), cmap="inferno",
        interpolation="bilinear", origin="upper",
        vmin=args.min_temp if args.min_temp is not None else 20.0,
        vmax=args.max_temp if args.max_temp is not None else 40.0,
    )
    colorbar = figure.colorbar(thermal_plot, ax=thermal_axis, fraction=0.046,
                               pad=0.04)
    colorbar.set_label("Temperature (C)")
    thermal_axis.set_title("红外热力图（等待数据）")
    thermal_axis.set_xlabel("MLX90640 X")
    thermal_axis.set_ylabel("MLX90640 Y")
    fusion_plot = fusion_axis.imshow(blank_photo, interpolation="nearest")
    fusion_axis.set_title("双光融合图（等待数据）")
    fusion_axis.set_axis_off()

    led = Circle((0.055, 0.91), 0.018, transform=figure.transFigure,
                 facecolor="#555555", edgecolor="#222222", linewidth=2)
    figure.add_artist(led)
    fire_text = figure.text(
        0.08, 0.91, "火情状态：未检测到火情", va="center", fontsize=13,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#e8f5e9",
              "edgecolor": "#2e7d32"},
    )
    location_text = figure.text(
        0.43, 0.91, "火情位置：无", va="center", fontsize=13,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#f5f5f5",
              "edgecolor": "#616161"},
    )
    connection_text = figure.text(
        0.08, 0.84, "通信状态：正在启动", va="center", fontsize=10,
        color="#37474f",
    )
    detector_text = figure.text(
        0.43, 0.84,
        "火情判定：STM32H7 端模型（PC 仅接收和显示）",
        va="center", fontsize=10, color="#6d4c41",
    )

    labels = ["可见光图像", "红外热力图", "双光融合图"]
    display_axes = [photo_axis, thermal_axis, fusion_axis]
    checks_axis = figure.add_axes([0.05, 0.045, 0.23, 0.13])
    checks = CheckButtons(checks_axis, labels, [True, True, True])
    checks_axis.set_title("图像显示开关", fontsize=10)
    mute_axis = figure.add_axes([0.82, 0.075, 0.12, 0.06])
    mute_button = Button(mute_axis, "警报消音", color="#eeeeee",
                         hovercolor="#ffcdd2")

    last = {
        "sequence": None, "received_at": None, "fps": 0.0,
        "photo_sequence": None,
        "fire_event_count": 0,
        "fire_active": False, "fire_started_at": None,
        "alarm_muted": False, "fire_photo_sequence": None,
        "latest_photo": None, "frozen_thermal": None,
        "peak_temperature": float("nan"), "fire_location": "未知",
    }

    def toggle_image(label: str) -> None:
        index = labels.index(label)
        display_axes[index].set_visible(checks.get_status()[index])
        figure.canvas.draw_idle()

    def mute_alarm(_event) -> None:
        alarm.silence()
        last["alarm_muted"] = True
        mute_button.label.set_text("已消音")
        figure.canvas.draw_idle()

    checks.on_clicked(toggle_image)
    mute_button.on_clicked(mute_alarm)

    def activate_fire(thermal: np.ndarray, photo: np.ndarray | None,
                      photo_sequence: int | None) -> None:
        """响应 STM32 的火情快照；这里只更新界面，不发送任何命令。"""
        valid = np.where(np.isfinite(thermal), thermal, -np.inf)
        flat_index = int(np.argmax(valid))
        row, column = np.unravel_index(flat_index, thermal.shape)
        peak = float(valid[row, column])
        last["fire_active"] = True
        last["fire_started_at"] = time.monotonic()
        last["frozen_thermal"] = thermal.copy()
        last["latest_photo"] = None if photo is None else photo.copy()
        last["fire_photo_sequence"] = photo_sequence
        last["peak_temperature"] = peak
        last["fire_location"] = (
            describe_fire_location(row, column, *thermal.shape)
            if np.isfinite(peak) else "未知"
        )
        last["alarm_muted"] = False
        mute_button.label.set_text("警报消音")
        alarm.start()

    def clear_fire() -> None:
        last["fire_active"] = False
        last["fire_started_at"] = None
        last["fire_photo_sequence"] = None
        last["frozen_thermal"] = None
        last["latest_photo"] = None
        alarm.silence()
        last["alarm_muted"] = False
        mute_button.label.set_text("警报消音")

    def update(_frame_number: int):
        """读取最新数据，并按 STM32 火情事件刷新三路图像。"""
        frame, state = receiver.snapshot()
        photo, photo_sequence = receiver.photo_snapshot()
        fire_frame, fire_event_count = receiver.fire_snapshot()
        live_thermal = None

        # 火情事件单独保存，避免 0x03 快照被紧随其后的实时帧覆盖而漏报。
        if (fire_frame is not None
                and fire_event_count != last["fire_event_count"]):
            last["fire_event_count"] = fire_event_count
            activate_fire(
                temperatures_from_frame(fire_frame, args.flip_x),
                photo, photo_sequence,
            )
        if frame is None:
            thermal_axis.set_title(f"红外热力图（{state}）")
        elif frame.sequence != last["sequence"]:
            live_thermal = temperatures_from_frame(frame, args.flip_x)
            # 根据相邻帧到达时间计算帧率，并进行简单低通平滑。
            if last["received_at"] is not None:
                delta = frame.received_at - last["received_at"]
                if delta > 0:
                    instant_fps = 1.0 / delta
                    last["fps"] = (instant_fps if last["fps"] == 0 else
                                   0.8 * last["fps"] + 0.2 * instant_fps)
            last["received_at"] = frame.received_at
            last["sequence"] = frame.sequence

            if not last["fire_active"]:
                thermal_plot.set_data(live_thermal)
                fusion_plot.set_data(build_fusion_image(photo, live_thermal))
            if (args.save_every > 0
                    and frame.frame_type == TEMPERATURE_FRAME
                    and frame.sequence % args.save_every == 0):
                save_frame(args.save_dir, live_thermal, frame=frame)

        if photo is not None and photo_sequence != last["photo_sequence"]:
            last["photo_sequence"] = photo_sequence
            last["latest_photo"] = photo.copy()
            last["fire_photo_sequence"] = photo_sequence
            photo_plot.set_data(photo)

        if (last["fire_active"] and last["fire_started_at"] is not None
                and time.monotonic() - last["fire_started_at"]
                >= args.fire_display_seconds):
            clear_fire()
            if frame is not None:
                live_thermal = temperatures_from_frame(frame, args.flip_x)
                thermal_plot.set_data(live_thermal)
                fusion_plot.set_data(build_fusion_image(photo, live_thermal))
            if photo is not None:
                photo_plot.set_data(photo)

        if last["fire_active"]:
            frozen_thermal = last["frozen_thermal"]
            live_photo = photo if photo is not None else last["latest_photo"]
            if live_photo is not None:
                photo_plot.set_data(live_photo)
                photo_axis.set_title(f"可见光图像（实时显示｜fire 报警｜seq {photo_sequence}）")
            else:
                photo_axis.set_title("可见光图像（实时显示｜等待图像）")
            thermal_plot.set_data(frozen_thermal)
            fusion_plot.set_data(build_fusion_image(live_photo,
                                                     frozen_thermal))
            thermal_axis.set_title("红外热力图（火情锁定）")
            fusion_axis.set_title("双光融合图（火情锁定）")
            led.set_facecolor("#f44336")
            led.set_edgecolor("#ffeb3b")
            peak = last["peak_temperature"]
            peak_text = f"{peak:.1f} ℃" if np.isfinite(peak) else "未知"
            fire_text.set_text(f"火情状态：STM32 已识别到 fire  最高温 {peak_text}")
            fire_text.get_bbox_patch().set_facecolor("#ffebee")
            fire_text.get_bbox_patch().set_edgecolor("#c62828")
            location_text.set_text(f"火情位置：{last['fire_location']}")
            elapsed = time.monotonic() - last["fire_started_at"]
            remaining = max(0.0, args.fire_display_seconds - elapsed)
            connection_text.set_text(
                f"通信状态：{state}；STM32 停车巡检倒计时 {remaining:.1f} 秒"
            )
        else:
            led.set_facecolor("#555555")
            led.set_edgecolor("#222222")
            fire_text.set_text("火情状态：未检测到火情")
            fire_text.get_bbox_patch().set_facecolor("#e8f5e9")
            fire_text.get_bbox_patch().set_edgecolor("#2e7d32")
            location_text.set_text("火情位置：无")
            connection_text.set_text(f"通信状态：{state}")
            if live_thermal is not None:
                valid = live_thermal[np.isfinite(live_thermal)]
                if valid.size:
                    data_min = float(np.min(valid))
                    data_max = float(np.max(valid))
                    if data_max - data_min < 1.0:
                        data_min -= 0.5
                        data_max += 0.5
                    thermal_plot.set_clim(
                        args.min_temp if args.min_temp is not None else data_min,
                        args.max_temp if args.max_temp is not None else data_max,
                    )
                    thermal_axis.set_title(
                        f"红外热力图 | {last['fps']:.1f} fps | "
                        f"min {data_min:.1f} ℃ | max {data_max:.1f} ℃"
                    )
            if photo is not None:
                photo_axis.set_title(f"可见光图像 seq {photo_sequence}")
            fusion_axis.set_title("双光融合图（实时预览）")
        return (thermal_plot, photo_plot, fusion_plot, led, fire_text,
                location_text, connection_text, detector_text)

    animation = FuncAnimation(
        figure, update, interval=40, blit=False, cache_frame_data=False
    )
    _ = animation
    window_closed = threading.Event()
    figure.canvas.mpl_connect(
        "close_event", lambda _event: window_closed.set()
    )
    try:
        # PyCharm 的 SciView 后端中 plt.show() 可能立即返回，显式运行事件
        # 循环可确保 TCP 接收线程持续工作，直到关闭窗口或手动停止程序。
        plt.show(block=False)
        while not window_closed.is_set():
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        alarm.close()
        receiver.stop()


def self_test() -> None:
    """验证热图协议以及 OV2640 照片分片重组和 BMP 保存。"""
    values = [2000 + index % 500 for index in range(32 * 24)]
    payload = struct.pack("<768h", *values)
    header = MAGIC + struct.pack(
        "<BBBBIIHhh", VERSION, TEMPERATURE_FRAME, 32, 24, 7, 1234,
        len(payload), min(values), max(values)
    )
    packet = header + payload
    packet += struct.pack("<H", crc16_ccitt_false(packet[4:]))
    parser = FrameParser()
    frames: list[Frame] = []
    # 故意加入前导噪声并切成三段，模拟真实网络分包。
    for chunk in (b"noise" + packet[:19], packet[19:700], packet[700:]):
        frames.extend(parser.feed(chunk))
    assert len(frames) == 1
    assert frames[0].sequence == 7
    rotated = temperatures_from_frame(frames[0], False)
    assert rotated.shape == (32, 24)
    assert np.isclose(rotated[0, 0], 20.31)

    # 构造一张纯红 RGB565 照片，按 STM32 相同的 4096 字节大小分片。
    photo_payload = b"\xF8\x00" * (PHOTO_RGB565_SIZE // 2)
    chunks = [
        photo_payload[offset:offset + MAX_PAYLOAD_SIZE]
        for offset in range(0, len(photo_payload), MAX_PAYLOAD_SIZE)
    ]
    with tempfile.TemporaryDirectory() as temporary_directory:
        receiver = Receiver(
            "127.0.0.1", 0, Path(temporary_directory),
            Path(temporary_directory) / "thermal", False,
        )
        for index, chunk in enumerate(chunks):
            receiver._handle_frame(Frame(
                frame_type=PHOTO_CHUNK_FRAME,
                width=PHOTO_WIDTH,
                height=PHOTO_HEIGHT,
                sequence=9,
                timestamp_ms=1234,
                minimum_centi_c=index,
                maximum_centi_c=len(chunks),
                payload=chunk,
                received_at=time.monotonic(),
            ))
        photo, sequence = receiver.photo_snapshot()
        assert receiver.photos_received == 1 and sequence == 1
        assert photo is not None and tuple(photo[0, 0]) == (255, 0, 0)
        assert (Path(temporary_directory) / "ov2640_000001.bmp").exists()
        thermal_directory = Path(temporary_directory) / "thermal"
        # 模拟 STM32 模型识别 fire 后发送 0x03 火情热图快照。
        fire_frame = dataclasses.replace(
            frames[0], frame_type=THERMAL_SNAPSHOT_FRAME
        )
        receiver._handle_frame(fire_frame)
        received_fire, fire_event_count = receiver.fire_snapshot()
        assert received_fire is fire_frame and fire_event_count == 1
        image = temperatures_from_frame(fire_frame, False)
        assert (thermal_directory / "mlx90640_000001.npy").exists()
        assert (thermal_directory / "mlx90640_000001.png").exists()

        fire_image = image.copy()
        fire_image[4, 20] = 85.0
        assert "右侧" in describe_fire_location(4, 20, *fire_image.shape)
        fused = build_fusion_image(photo, fire_image)
        assert fused.shape == (PHOTO_HEIGHT, PHOTO_WIDTH, 3)
        assert fused.dtype == np.uint8
    print("Protocol self-test passed")


def parse_args() -> argparse.Namespace:
    """解析监听地址、显示方式、色温范围和数据保存参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bind", default="0.0.0.0",
        help="local address used by the PC TCP server"
    )
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--min-temp", type=float)
    parser.add_argument("--max-temp", type=float)
    parser.add_argument(
        "--fire-display-seconds", type=float, default=10.0,
        help="seconds to keep an STM32 fire event locked in the GUI",
    )
    parser.add_argument(
        "--no-alarm", action="store_true",
        help="disable the audible PC alarm while keeping visual alerts",
    )
    parser.add_argument(
        "--alarm-sound", type=Path,
        default=Path(__file__).resolve().parent / "the_sound_of_fire_alarm.mp3",
        help="MP3 file played repeatedly while a fire alarm is active",
    )
    parser.add_argument(
        "--save-dir", type=Path,
        default=Path(__file__).resolve().parent / "thermal",
        help="directory used to save thermal PNG and NPY files",
    )
    parser.add_argument(
        "--save-every", type=int, default=0,
        help="also auto-save every N live thermal frames; 0 disables it",
    )
    parser.add_argument(
        "--photo-dir", type=Path,
        default=Path(__file__).resolve().parent / "captures",
        help="directory used to save OV2640 BMP photos",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.save_every < 0:
        parser.error("--save-every must be zero or greater")
    if args.fire_display_seconds <= 0:
        parser.error("--fire-display-seconds must be positive")
    if not args.no_alarm and not args.alarm_sound.is_file():
        parser.error(f"alarm sound file not found: {args.alarm_sound}")
    return args


def main() -> None:
    """程序入口：启动 TCP 接收线程，再运行无界面或图形界面。"""
    args = parse_args()
    if args.self_test:
        self_test()
        return
    receiver = Receiver(
        args.bind, args.port, args.photo_dir,
        args.save_dir, args.flip_x,
    )
    receiver.start()
    try:
        run_headless(receiver, args) if args.headless else run_gui(receiver, args)
    finally:
        receiver.stop()
        receiver.join(timeout=2.0)


if __name__ == "__main__":
    main()
