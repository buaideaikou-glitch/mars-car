"""
Fashion Star 舵机 UART 总线库。

控制指令采用无返回报文的同步发送方案：
- ServoBus.set_angles() 使用 0x0c 速度型单圈位置控制。
- 多舵机同时控制时使用 0x19 同步帧封装 0x0c。

位置查询采用 0x16 数据监控；多个 ID 时通过 0x19 同步帧一次下发、
一次收齐各舵机响应，避免逐个轮询。

作者 王笑
日期 20260528
"""

import time

try:
    import ustruct
except ImportError:
    import struct as ustruct

from robot_config import (
    SERVO_UART_BAUD,
    clamp,
)

_SERVO_DEFAULT_SPEED_DEG_S = 60.0
_SERVO_ACCEL_TIME_MS = 100
_SERVO_DECEL_TIME_MS = 100
_SERVO_BUS_GAP_MS = 5
_SERVO_TX_TIME_SCALE_PERCENT = 140
_SERVO_READ_BASE_MS = 3
_SERVO_READ_PER_SERVO_MS = 2
_SERVO_READ_TIMEOUT_MARGIN_MS = 30


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(ms))
    else:
        time.sleep(float(ms) / 1000.0)


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_add(ticks, delta):
    if hasattr(time, "ticks_add"):
        return time.ticks_add(ticks, int(delta))
    return ticks + int(delta)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def clear_uart_input(uart, max_ms=20):
    if uart is None:
        return
    try:
        deadline = ticks_add(ticks_ms(), max_ms)
        while uart.any():
            uart.read(uart.any())
            sleep_ms(1)
            if ticks_diff(deadline, ticks_ms()) <= 0:
                break
    except Exception:
        pass


def _uart_tx_time_ms(byte_count, baud=SERVO_UART_BAUD):
    if byte_count <= 0:
        return 0
    scale = max(100, int(_SERVO_TX_TIME_SCALE_PERCENT))
    numerator = int(byte_count) * 10 * 1000 * scale
    denominator = int(baud) * 100
    return (numerator + denominator - 1) // denominator


def servo_bus_gap(tx_bytes=0):
    sleep_ms(_SERVO_BUS_GAP_MS + _uart_tx_time_ms(tx_bytes))


def servo_tx_wait(tx_bytes):
    sleep_ms(_uart_tx_time_ms(tx_bytes))


def _monitor_read_timeout_ms(servo_count):
    count = max(1, int(servo_count))
    return int(
        _SERVO_READ_BASE_MS +
        (count - 1) * _SERVO_READ_PER_SERVO_MS +
        _SERVO_READ_TIMEOUT_MARGIN_MS
    )


# =============================================================================
# Fashion Star 二进制 UART 总线
# =============================================================================

SERVO_HDR_TX = bytes([0x12, 0x4C])
SERVO_HDR_RX = bytes([0x05, 0x1C])
SERVO_CMD_PING = 0x01
SERVO_CMD_SET_ANGLE_SPEED = 0x0C
SERVO_CMD_MONITOR = 0x16
SERVO_CMD_RESET_TURNS = 0x11
SERVO_CMD_STOP = 0x18
SERVO_CMD_SYNC = 0x19
SERVO_STOP_RELEASE = 0x10
SERVO_STOP_LOCK = 0x11

_MONITOR_PER_SERVO_LEN = 1
_MONITOR_RESP_DATA_LEN = 16
_MONITOR_RESP_PKT_LEN = 4 + _MONITOR_RESP_DATA_LEN + 1
_MONITOR_POSITION_OFF = 14
_MONITOR_TURNS_OFF = 18


def servo_checksum(data):
    return sum(data) & 0xFF


def build_servo_packet(cmd_id, content):
    packet = bytearray(SERVO_HDR_TX)
    packet.append(cmd_id & 0xFF)
    packet.append(len(content) & 0xFF)
    packet.extend(content)
    packet.append(servo_checksum(packet))
    return packet


class ServoBus:
    def __init__(self, uart):
        self.uart = uart

    def _angle_speed_content(self, servo_id, angle_deg,
                             speed_deg_s=_SERVO_DEFAULT_SPEED_DEG_S,
                             accel_ms=_SERVO_ACCEL_TIME_MS,
                             decel_ms=_SERVO_DECEL_TIME_MS,
                             power=0):
        angle_deg = clamp(float(angle_deg), -180.0, 180.0)
        position = int(round(angle_deg * 10.0))
        speed = int(round(abs(float(speed_deg_s)) * 10.0))
        speed = clamp(speed, 1, 65535)
        accel_ms = clamp(int(accel_ms), 20, 65535)
        decel_ms = clamp(int(decel_ms), 20, 65535)
        content = bytearray()
        content.append(int(servo_id) & 0xFF)
        content.extend(ustruct.pack("<h", position))
        content.extend(ustruct.pack("<H", speed))
        content.extend(ustruct.pack("<H", accel_ms))
        content.extend(ustruct.pack("<H", decel_ms))
        content.extend(ustruct.pack("<H", clamp(int(power), 0, 65535)))
        return content

    def set_angle(self, servo_id, angle_deg,
                  speed_deg_s=_SERVO_DEFAULT_SPEED_DEG_S,
                  accel_ms=_SERVO_ACCEL_TIME_MS,
                  decel_ms=_SERVO_DECEL_TIME_MS,
                  power=0):
        self.set_angles(
            ((servo_id, angle_deg),),
            speed_deg_s=speed_deg_s,
            accel_ms=accel_ms,
            decel_ms=decel_ms,
            power=power,
        )

    def set_angles(self, targets,
                   speed_deg_s=_SERVO_DEFAULT_SPEED_DEG_S,
                   accel_ms=_SERVO_ACCEL_TIME_MS,
                   decel_ms=_SERVO_DECEL_TIME_MS,
                   power=0):
        items = list(targets)
        if not items:
            return
        if len(items) == 1:
            servo_id, angle_deg = items[0]
            content = self._angle_speed_content(
                servo_id,
                angle_deg,
                speed_deg_s,
                accel_ms,
                decel_ms,
                power,
            )
            clear_uart_input(self.uart)
            packet = build_servo_packet(SERVO_CMD_SET_ANGLE_SPEED, content)
            self.uart.write(packet)
            servo_bus_gap(len(packet))
            return

        # 0x19 同步命令：多个 0x0c 速度型角度控制内容合成一帧，接收完后同时执行。
        content = bytearray()
        content.append(SERVO_CMD_SET_ANGLE_SPEED)
        content.append(11)
        content.append(len(items) & 0xFF)
        for servo_id, angle_deg in items:
            content.extend(
                self._angle_speed_content(
                    servo_id,
                    angle_deg,
                    speed_deg_s,
                    accel_ms,
                    decel_ms,
                    power,
                )
            )
        clear_uart_input(self.uart)
        packet = build_servo_packet(SERVO_CMD_SYNC, content)
        self.uart.write(packet)
        servo_bus_gap(len(packet))

    def ping(self, servo_id, timeout_ms=40):
        content = bytearray([int(servo_id) & 0xFF])
        try:
            clear_uart_input(self.uart)
            packet = build_servo_packet(SERVO_CMD_PING, content)
            self.uart.write(packet)
            servo_tx_wait(len(packet))
            expected_id = bytes([int(servo_id) & 0xFF])
            deadline = ticks_add(ticks_ms(), timeout_ms)
            data = b""
            while ticks_diff(deadline, ticks_ms()) > 0:
                n = self.uart.any()
                if n:
                    chunk = self.uart.read(n)
                    if chunk:
                        data += chunk
                        if SERVO_HDR_RX in data and expected_id in data:
                            servo_bus_gap()
                            return True
                sleep_ms(2)
        except Exception:
            pass
        servo_bus_gap()
        return False

    def read_angle(self, servo_id, timeout_ms=None):
        """读取单个舵机角度（0x16 数据监控）。"""
        results = self.read_angles((servo_id,), timeout_ms=timeout_ms)
        if results:
            return results[0][1]
        return None

    def read_angles(self, servo_ids, timeout_ms=None):
        """
        批量读取舵机角度（0x16 数据监控）。
        多个 ID 时通过 0x19 同步帧一次查询，再收齐各舵机 0x16 响应。
        """
        ids = [int(sid) & 0xFF for sid in servo_ids]
        if not ids:
            return []
        if timeout_ms is None:
            timeout_ms = _monitor_read_timeout_ms(len(ids))
        try:
            self._send_monitor_query(ids)
            return self._wait_monitor_responses(ids, timeout_ms)
        except Exception:
            pass
        servo_bus_gap()
        return []

    def _send_monitor_query(self, servo_ids):
        clear_uart_input(self.uart)
        if len(servo_ids) == 1:
            content = bytearray([servo_ids[0]])
            packet = build_servo_packet(SERVO_CMD_MONITOR, content)
        else:
            content = bytearray()
            content.append(SERVO_CMD_MONITOR)
            content.append(_MONITOR_PER_SERVO_LEN)
            content.append(len(servo_ids) & 0xFF)
            for servo_id in servo_ids:
                content.append(servo_id)
            packet = build_servo_packet(SERVO_CMD_SYNC, content)
        self.uart.write(packet)
        servo_tx_wait(len(packet))

    def _wait_monitor_responses(self, expected_ids, timeout_ms):
        expected = set(expected_ids)
        seen = {}
        deadline = ticks_add(ticks_ms(), int(timeout_ms))
        data = b""
        while ticks_diff(deadline, ticks_ms()) > 0 and len(seen) < len(expected):
            n = self.uart.any()
            if n:
                chunk = self.uart.read(n)
                if chunk:
                    data += chunk
                    for sid, angle_deg in self._parse_monitor_packets(data, expected):
                        if sid not in seen:
                            seen[sid] = angle_deg
                    if len(seen) >= len(expected):
                        break
            else:
                sleep_ms(2)
        servo_bus_gap()
        return [(sid, seen[sid]) for sid in expected_ids if sid in seen]

    def reset_turns(self, servo_id):
        """重置多圈计数，将当前绝对位置重新记录为当前角度。"""
        content = bytearray([int(servo_id) & 0xFF])
        clear_uart_input(self.uart)
        packet = build_servo_packet(SERVO_CMD_RESET_TURNS, content)
        self.uart.write(packet)
        servo_bus_gap(len(packet))

    def reset_turns_polling(self, servo_ids):
        """
        逐个舵机重置圈数。

        按协议建议，先停止/释放舵机，再发送重置圈数，最后锁住当前位置。
        """
        for servo_id in servo_ids:
            self.disable(servo_id)
            sleep_ms(20)
            self.reset_turns(servo_id)
            sleep_ms(20)
            self.enable(servo_id)
            sleep_ms(20)

    def lock_all(self, servo_ids):
        """逐个锁住当前位置，保持舵机锁力。"""
        for servo_id in servo_ids:
            self.enable(servo_id)
            sleep_ms(20)

    def _parse_monitor_packets(self, data, expected_ids=None):
        expected = None if expected_ids is None else set(int(sid) for sid in expected_ids)
        angles = []
        seen = set()
        start = data.find(SERVO_HDR_RX)
        while start >= 0 and len(data) >= start + _MONITOR_RESP_PKT_LEN:
            cmd_id = data[start + 2]
            length = data[start + 3]
            end = start + 4 + length + 1
            if len(data) >= end and length == _MONITOR_RESP_DATA_LEN:
                pkt = data[start:end]
                if (
                    servo_checksum(pkt[:-1]) == pkt[-1]
                    and cmd_id == SERVO_CMD_MONITOR
                ):
                    sid = int(pkt[4])
                    if (expected is None or sid in expected) and sid not in seen:
                        # 协议 21.3：position 为当前绝对角度，单位 0.1°（有符号）。
                        # 不要与 turns 叠加；turns 为独立圈数信息。
                        position = ustruct.unpack(
                            "<i",
                            pkt[_MONITOR_POSITION_OFF:_MONITOR_TURNS_OFF],
                        )[0]
                        angle_deg = position / 10.0
                        angles.append((sid, angle_deg))
                        seen.add(sid)
            start = data.find(SERVO_HDR_RX, start + 1)
        return angles

    def enable(self, servo_id, power=0):
        self._stop(servo_id, SERVO_STOP_LOCK, power)

    def disable(self, servo_id, power=0):
        self._stop(servo_id, SERVO_STOP_RELEASE, power)

    def _stop(self, servo_id, mode, power=0):
        content = bytearray()
        content.append(int(servo_id) & 0xFF)
        content.append(mode & 0xFF)
        content.extend(ustruct.pack("<H", clamp(int(power), 0, 65535)))
        clear_uart_input(self.uart)
        packet = build_servo_packet(SERVO_CMD_STOP, content)
        self.uart.write(packet)
        servo_bus_gap(len(packet))


