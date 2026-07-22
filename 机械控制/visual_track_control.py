"""
视觉追踪闭环控制。

通过 UART 接收 K210 视觉数据，实现比例控制自动对齐 3×3 色块。

状态机：
- SEARCH_QR：慢速前进，寻找二维码获取目标颜色
- APPROACH：已知目标颜色，前进直到 9 个块全部可见
- WAIT_L1：停车等待 PS2 L1 按键触发精调
- ALIGN：比例控制对齐，直到 XY 偏移均在死区内
- DONE：对齐完成，退出

作者 王笑（基于视觉追踪需求扩展）
日期 20260717
"""

import time
import re
import _thread

from chassis_control import LunarRover
from ps2_lib import PS2Controller, PS2Receiver
from robot_config import (
    PS2_DI, PS2_DO, PS2_CS, PS2_CLK,
    VISUAL_TRACK_SPEED,
    VISUAL_TRACK_KP_STEER,
    VISUAL_TRACK_DEAD_ZONE,
    VISUAL_TRACK_TARGET_AREA_MIN,
    VISUAL_TRACK_TARGET_AREA_MAX,
    VISUAL_TRACK_DATA_TIMEOUT_MS,
    DEFAULT_ACC_RAD_S2,
    clamp,
)

SEARCH_QR = 0
APPROACH = 1
WAIT_L1  = 2
ALIGN     = 3
DONE      = 4

_L1_MASK = 0x0400
_IMG_CENTER_X = 160
_IMG_CENTER_Y = 120


def _parse_colors_from_qr(qr_payload):
    """从 QR 码载荷解析目标颜色列表。支持逗号、空格、分号、下划线分隔。"""
    colors = re.split(r'[,;\s_]+', qr_payload.strip())
    return [c.lower() for c in colors if c]


def _parse_visual_data(data_str):
    """
    解析视觉数据字符串。
    格式：<C1> <N1> <C2> <N2> <C3> <N3> <CX> <CY> <AREA> <QR>
    返回 dict 或 None。
    """
    parts = data_str.split()
    if len(parts) < 9:
        return None

    try:
        c1, n1 = parts[0], int(parts[1])
        c2, n2 = parts[2], int(parts[3])
        c3, n3 = parts[4], int(parts[5])
        cx = float(parts[6])
        cy = float(parts[7])
        area = int(parts[8])
    except (ValueError, IndexError):
        return None

    qr = None
    if len(parts) >= 10 and parts[9] != "NONE":
        qr = parts[9].replace("_", " ")

    return {
        "colors": [(c1, n1), (c2, n2), (c3, n3)],
        "cx": cx,
        "cy": cy,
        "area": area,
        "qr": qr,
    }


def _count_target_blocks(vd, target_colors):
    """根据目标颜色列表统计色块总数。"""
    if not target_colors:
        return 0
    color_map = {}
    for c_name, c_count in vd["colors"]:
        color_map[c_name.lower()] = c_count
    total = 0
    for tc in target_colors:
        total += color_map.get(tc, 0)
    return total


def visual_track_loop(rover, camera_data, camera_uart):
    """视觉追踪主控制循环。"""

    state = SEARCH_QR
    target_colors = None
    last_data_ticks = time.ticks_ms()
    started = False

    ps2_ctrl = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
    ps2_ctrl.init_vibration()
    ps2 = PS2Receiver(ps2_ctrl, 30, True)
    ps2.start()

    print("[视觉追踪] 启动 — SEARCH_QR")
    rover.prepare()

    try:
        while True:
            raw = None
            if camera_data["value"] is not None:
                raw = camera_data["value"]
                camera_data["value"] = None
                if camera_uart is not None:
                    camera_uart.write("zdok")

            vd = None
            if raw is not None:
                vd = _parse_visual_data(raw)
                if vd is not None:
                    last_data_ticks = time.ticks_ms()

            # =========== data timeout ===========
            if time.ticks_diff(time.ticks_ms(), last_data_ticks) > VISUAL_TRACK_DATA_TIMEOUT_MS:
                rover.stop()
                print("[视觉追踪] 视觉数据超时，停车等待……")

            # =========== state machine ===========

            if state == SEARCH_QR:
                if vd is not None and vd["qr"] is not None:
                    target_colors = _parse_colors_from_qr(vd["qr"])
                    if target_colors:
                        print(f"[视觉追踪] QR获取目标颜色: {target_colors}")
                        state = APPROACH
                        continue  # 立刻用当前帧走下一状态

                # 慢速前进
                rover.drive(speed_rad_s=VISUAL_TRACK_SPEED * 0.5,
                            steer_angle_deg=0.0)

            elif state == APPROACH:
                if vd is None:
                    time.sleep_ms(20)
                    continue

                total = _count_target_blocks(vd, target_colors)
                if not started:
                    print(f"[视觉追踪] 接近中 目标块数={total}/9 CX={vd['cx']:.0f}")
                    started = True

                if total >= 9:
                    rover.stop()
                    print("[视觉追踪] 9 块已全部可见，等待 L1 触发精调")
                    state = WAIT_L1
                    started = False
                else:
                    offset_x = vd["cx"] - _IMG_CENTER_X
                    steer = clamp(VISUAL_TRACK_KP_STEER * offset_x, -30.0, 30.0)
                    rover.drive(speed_rad_s=VISUAL_TRACK_SPEED * 0.7,
                                steer_angle_deg=steer)

            elif state == WAIT_L1:
                snap = ps2.snapshot(max_age_ms=100)
                if snap is not None and snap.fresh and (snap.buttons & _L1_MASK):
                    print("[视觉追踪] L1 按下，开始自动对齐")
                    state = ALIGN
                    started = False
                time.sleep_ms(50)

            elif state == ALIGN:
                if vd is None:
                    time.sleep_ms(20)
                    continue

                offset_x = vd["cx"] - _IMG_CENTER_X
                offset_y = vd["cy"] - _IMG_CENTER_Y

                if not started:
                    print(f"[视觉追踪] 对齐中 offset_x={offset_x:.1f} offset_y={offset_y:.1f} area={vd['area']}")
                    started = True

                aligned_x = abs(offset_x) <= VISUAL_TRACK_DEAD_ZONE
                aligned_y = abs(offset_y) <= VISUAL_TRACK_DEAD_ZONE
                area_ok = VISUAL_TRACK_TARGET_AREA_MIN <= vd["area"] <= VISUAL_TRACK_TARGET_AREA_MAX

                if aligned_x and aligned_y and area_ok:
                    rover.stop()
                    print("[视觉追踪] 对齐完成，退出")
                    state = DONE
                    break
                elif aligned_x:
                    # X 已对准，纯前进
                    rover.drive(speed_rad_s=VISUAL_TRACK_SPEED * 0.3,
                                steer_angle_deg=0.0)
                else:
                    steer = clamp(VISUAL_TRACK_KP_STEER * offset_x, -45.0, 45.0)
                    rover.drive(speed_rad_s=VISUAL_TRACK_SPEED * 0.3,
                                steer_angle_deg=steer)

            elif state == DONE:
                break

            time.sleep_ms(30)

    except Exception as e:
        print("[视觉追踪] 异常:", e)

    finally:
        rover.stop()
        ps2.stop()

    print("[视觉追踪] 结束")
