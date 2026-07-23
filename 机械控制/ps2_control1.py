"""
PS2 遥控业务控制逻辑。

本文件负责把 PS2 按键和摇杆映射到底盘、相机和机械臂动作。
ps2_lib.py 只负责手柄底层读取和安全接收。

作者 王笑
日期 20260528
"""

import math
import time

from arm_control import ArmKinematicsError
from robot_config import (
    MAX_MOTOR_RPM,
    MAX_STEER_ANGLE_DEG,
    PIVOT_SPEED_SCALE,
    clamp,
)

_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_MAX_PIVOT_RAD_S = _MAX_MOTOR_RAD_S * PIVOT_SPEED_SCALE
_ARM_JOG_COMMAND_DELAY_MS = 50
_ARM_JOG_STEP_DEG = 8
_last_arm_error_key = None
_last_arm_error_ms = 0

# ===== 抓取模式相关变量 =====
grab_mode = False          # 是否处于抓取模式
grab_cooldown = False      # 抓取冷却
grab_cooldown_time = 0     # 冷却开始时间
row = 1.5
backup_mode = False        # 后退模式：True=正在后退拉开dy
backup_done = False        # 本周期已后退过，防止对齐时再次触发
reverse_mode = False       # 后退避让模式（带滞回），防止dy≈10时转向角跳变震荡

# ===== 自动循迹模式相关变量 =====
line_track_mode = False    # 是否处于自动循迹模式
line_track_lost = False    # 绿线是否丢失
line_track_lost_time = 0   # 丢失开始时间
_LINE_TRACK_SPEED = 0.5    # 循迹前进速度 (rad/s)
_LINE_TRACK_MAX_STEER = 90.0  # 循迹最大转向角 (deg)
_LINE_TRACK_DX_THRESHOLD = 15  # dx阈值，小于此值直行
_LINE_TRACK_LOST_TIMEOUT = 2.0  # 丢失超时自动退出 (秒)

# 抓取/放置过程中使用的固定参数
_GRIPPER_SERVO_ID = 15       # 夹爪舵机 ID（与 L1/R1 手动控制一致）
_GRIPPER_OPEN_DEG = 0.0      # 夹爪松开角度（对应 L1）
_GRIPPER_CLOSE_DEG = 80.0    # 夹爪夹紧角度（对应 R1）
_ARM_GRAB_SPEED = 60.0       # 抓取过程中机械臂运动速度 (deg/s)
_GRIPPER_SETTLE_MS = 1200    # 夹爪动作后的等待时间
_ARM_SERVO_IDS = (7, 9, 10, 11)  # 机械臂四个关节舵机 ID


def _move_arm_servos(rover, targets, speed_deg_s=_ARM_GRAB_SPEED, extra_ms=400):
    """
    按原始舵机角度（与 START 打印一致）驱动 7/9/10/11 号机械臂舵机，
    并按「最大角差 / 速度」估算等待时间，确保到位后再返回。
    targets: ((servo_id, angle_deg), ...) 形式的原始舵机角度。
    """
    bus = rover.servo_control.servo_bus
    current = dict(bus.read_angles(_ARM_SERVO_IDS))
    max_delta = 0.0
    for servo_id, angle_deg in targets:
        cur = current.get(servo_id)
        if cur is not None:
            max_delta = max(max_delta, abs(float(angle_deg) - float(cur)))
    bus.set_angles(targets, speed_deg_s=speed_deg_s)
    wait_ms = int(max_delta / max(float(speed_deg_s), 1.0) * 1000) + int(extra_ms)
    time.sleep_ms(wait_ms)


# ==============================================================================
# 核心摇杆数据处理函数
# ==============================================================================
def map_joystick(raw_val, center=128, deadzone=12):
    """
    【摇杆数据映射核心】
    将摇杆的原始 ADC 数据 (通常为 0-255) 转换为 -100 到 100 的百分比数值。
    
    参数说明:
    - raw_val: 手柄底层读取到的原始摇杆数据 (0~255)
    - center: 摇杆的物理中位值 (默认128)
    - deadzone: 死区范围，摇杆在这个范围内的微小偏移会被忽略，防止摇杆回中不良导致漂移
    """
    # 1. 计算偏移量 (例如 128 - 128 = 0; 255 - 128 = 127)
    offset = int(raw_val) - center
    
    # 2. 死区过滤：如果偏移量在死区范围内，说明没有有效拨动，直接返回 0
    if abs(offset) <= deadzone:
        return 0
        
    # 3. 确定方向：正向推为 1，反向拉为 -1
    sign = 1 if offset > 0 else -1
    
    # 4. 计算有效活动区间 (127 - 12 = 115)
    active_range = 127.0 - deadzone
    
    # 5. 扣除死区后，将实际偏移量映射到 0~100 的百分比，并附加上方向符号
    mapped = int(((abs(offset) - deadzone) / active_range) * 100.0) * sign
    
    # 6. 安全限制：确保最终输出严格在 -100 到 100 之间
    return clamp(mapped, -100, 100)


def small_motion(value, threshold):
    # 过滤微小动作，如果计算出的运动增量小于设定阈值，则直接归零
    return 0.0 if abs(value) < threshold else value


def button_pressed(data, btn):
    # 通过位与运算判断底层传来的复合数据中，某个特定按键是否被按下
    return (data & btn) == btn


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def print_arm_error(err):
    # 打印机械臂错误，并进行防刷屏处理（1秒内相同的错误只报一次）
    global _last_arm_error_key, _last_arm_error_ms
    now_ms = ticks_ms()
    key = (err.reason, err.message)
    if key == _last_arm_error_key and ticks_diff(now_ms, _last_arm_error_ms) < 1000:
        return
    _last_arm_error_key = key
    _last_arm_error_ms = now_ms
    print("机械臂目标无效：%s，%s" % (err.reason, err.message))


def sync_arm_control_state(rover):
    if rover.arm is None:
        return True
    try:
        rover.arm.sync_from_servos()
        rover.arm.sync_camera_from_servo()
    except ArmKinematicsError as err:
        print_arm_error(err)
        return False
    return True

# ==============================================================================
# 机械臂控制模式（处理传进来的摇杆数据）
# ==============================================================================
def handle_arm_control(rover, ps2, buttons, lx, ly, rx, ry):
    if rover.arm is None:
        return

    # 【L1 键】松开夹爪（0°），【R1 键】夹紧夹爪（80°）
    if button_pressed(buttons, ps2.PS2_BTN_L1):
        rover.arm.servo_control.set_reserve_servo_angle(15, 0.0, speed_deg_s=60.0)
        time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
        return
    if button_pressed(buttons, ps2.PS2_BTN_R1):
        rover.arm.servo_control.set_reserve_servo_angle(15, 80.0, speed_deg_s=60.0)
        time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
        return

    # 【O 键】复位机械臂、相机角度及预留舵机
    # 【O 键】复位机械臂、相机角度及预留舵机
    if button_pressed(buttons, ps2.PS2_BTN_CIRCLE):
        try:
            rover.arm.apply_initial_pose()
        except ArmKinematicsError as err:
            print_arm_error(err)
        try:
            rover.arm.jog_camera(-rover.arm.camera_angle_deg)
        except ArmKinematicsError as err:
            print_arm_error(err)
        try:
            rover.servo_control.init_reserve_servos()
        except Exception as err:
            print("预留舵机复位失败：", err)
        try:
            rover.servo_control.init_reserve_servos()
        except Exception as err:
            print("预留舵机复位失败：", err)
        return
    
    # 【十字键 左右】微调相机角度
    if button_pressed(buttons, ps2.PS2_BTN_LEFT):
        try:
            rover.arm.jog_camera(_ARM_JOG_STEP_DEG)
        except ArmKinematicsError as err:
            print_arm_error(err)
    if button_pressed(buttons, ps2.PS2_BTN_RIGHT):
        try:
            rover.arm.jog_camera(-_ARM_JOG_STEP_DEG)
        except ArmKinematicsError as err:
            print_arm_error(err)

    # 【摇杆数据读取与映射 - 机械臂模式】
    # 机械臂模式的死区设为 20（比底盘严格），避免从底盘切换时发生误碰
    left_y = map_joystick(ly, deadzone=20)
    # 取反适配机械臂 Roll 轴坐标系
    right_x = -map_joystick(rx, deadzone=20) 
    right_y = map_joystick(ry, deadzone=20)

    roll_delta = 0.0
    pitch1_delta = 0.0
    pitch2_delta = 0.0
    pitch3_delta = 0.0

    # 【将百分比 (-100~100) 转化为角度增量】
    # / 100.0 将其变为 -1.0 到 1.0 的比例系数，再乘以最大步进角度
    if right_x != 0:
        roll_delta = right_x / 100.0 * _ARM_JOG_STEP_DEG
    if right_y != 0:
        pitch1_delta = right_y / 100.0 * _ARM_JOG_STEP_DEG
    if left_y != 0:
        pitch2_delta = left_y / 100.0 * _ARM_JOG_STEP_DEG
        
    # 【十字键 上下】按照固定步进修改 Pitch3
    if button_pressed(buttons, ps2.PS2_BTN_UP):
        pitch3_delta += _ARM_JOG_STEP_DEG
    if button_pressed(buttons, ps2.PS2_BTN_DOWN):
        pitch3_delta -= _ARM_JOG_STEP_DEG

    # 使用 small_motion 过滤掉极微小变化 (小于 0.5度的杂波)
    roll_delta = small_motion(roll_delta, 0.5)
    pitch1_delta = small_motion(pitch1_delta, 0.5)
    pitch2_delta = small_motion(pitch2_delta, 0.5)
    pitch3_delta = small_motion(pitch3_delta, 0.5)

    if (roll_delta != 0.0 or pitch1_delta != 0.0 or
            pitch2_delta != 0.0 or pitch3_delta != 0.0):
        try:
            rover.arm.jog_joints(
                roll_delta,
                pitch1_delta,
                pitch2_delta,
                pitch3_delta,
            )
        except ArmKinematicsError as err:
            print_arm_error(err)

# ==============================================================================
# 主循环控制：演示如何从底层获取摇杆信息
# ==============================================================================
def ps2_loop(rover, ps2, data, serial):
    global grab_mode, grab_cooldown, grab_cooldown_time, backup_mode, backup_done, reverse_mode
    global line_track_mode, line_track_lost, line_track_lost_time
    
    print("PS2 控制：X失能，三角使能，R1停车，R2+右摇杆左右原地转向，L2+O机械臂回初始位并相机回0，L2+方向键左右控制相机，上下控制Pitch3，L2+左摇杆前后控制Pitch2，右摇杆前后控制Pitch1，右摇杆左右控制Roll。")
    print("方框键：切换自动抓取模式")
    print("❌键：切换自动循迹模式（沿绿线行走）")
    print("START键：打印 12/13/14/15 号舵机当前角度")
    arm_mode_active = False
    
    while True:
        # 【第一步：触发底层更新】要求底层库发起一次 SPI 通信，读取手柄当前状态
        ps2.update()
        #print(f"🔍 调试：data['value'] = {data['value']}")
        # ===== 处理摄像头数据（抓取模式） =====
        
        # 【第二步：获取摇杆信息的关键快照】
        fresh, buttons, lx, ly, rx, ry, _ = ps2.snapshot()
        
        # 如果获取数据失败 (手柄断开或通讯异常)，停止动作并重新尝试获取
        if not fresh:
            rover.stop()
            arm_mode_active = False
            continue

        # 【SELECT 键】退出PS2控制
        if button_pressed(buttons, ps2.PS2_BTN_SELECT):
            rover.stop()
            print("SELECT：退出 PS2 控制。")
            break

        # 【START 键】打印 12/13/14/15 号舵机当前角度
        # 走底层 servo_bus.read_angles 一次性查询（不看 robot_config 登记，没接的 ID 显示无响应）
        if button_pressed(buttons, ps2.PS2_BTN_START):
            result = dict(rover.servo_control.servo_bus.read_angles((7, 9, 10, 11, 14)))
            print("===== 舵机角度查询 =====")
            for _sid in (7, 9, 10, 11, 14):
                _ang = result.get(_sid)
                if _ang is None:
                    print("  舵机 %d: 无响应" % _sid)
                else:
                    print("  舵机 %d: %.1f°" % (_sid, _ang))
            time.sleep_ms(300)  # 防抖，避免按住时刷屏
            continue
    
        # 【方框键】切换抓取模式
        if button_pressed(buttons, ps2.PS2_BTN_SQUARE):
            grab_mode = not grab_mode
            if grab_mode:
                print("🎯 L2+三角键按下，进入抓取模式")
                rover.stop()
                grab_cooldown = False
                grab_cooldown_time = 0
                data["value"] = None  # 清除残留的finish信号
                if rover.arm is not None:
                    try:
                        rover.arm.sync_camera_from_servo()
                    except ArmKinematicsError as err:
                        print_arm_error(err)
            else:
                print("⏹️ 退出抓取模式")
                rover.stop()
            time.sleep_ms(200)  # 防抖
            continue
        
        # ==========================================================
        # 抓取模式的自动逻辑
        # ==========================================================
        if grab_mode:
            # 检查finish信号（任何时候都响应，包括冷却期间）
            camera_val = data["value"]
            if camera_val is not None and camera_val == "finish":
                data["value"] = None
                print("📢 收到finish，自动退出抓取模式")
                grab_mode = False
                grab_cooldown = False
                rover.stop()
                continue

            if grab_cooldown:
                if time.time() - grab_cooldown_time < 1.0:
                    pass
                else:
                    grab_cooldown = False
                    print("🔄 冷却结束")
            
            if not grab_cooldown:
                serial_data = data["value"]
                if serial_data is not None:
                    data["value"] = None
                    parts = serial_data.strip().split()
                    if len(parts) == 2:
                        try:
                            dx = int(parts[0])
                            dy = int(parts[1])
                            
                            # ===== 阶段0：丢失巡游 =====
                            if dx == 999 and dy == 999:
                                backup_mode = False
                                backup_done = False
                                reverse_mode = False
                                print("🔍 目标丢失，巡游中...")
                                row *= -1
                                rover.servo_control.set_steering_angles(90.0, 90.0, 90.0, 90.0, 90.0, 90.0)
                                rover.drive(speed_rad_s= row+0.5, steer_angle_deg=0.0)
                                time.sleep(1)
                                rover.stop()
                                rover.servo_control.set_steering_angles(0, 0, 0, 0, 0, 0)

                            # ===== 阶段0.5：后退模式（dy过小，持续后退直到dy<-10）=====
                            elif backup_mode:
                                if dy < -10:
                                    backup_mode = False
                                    backup_done = True
                                    print("✅ 后退完成 dy={:.0f}, 恢复追踪".format(dy))
                                else:
                                    print("🔄 后退拉开 dy={:.0f}".format(dy))
                                    rover.drive(speed_rad_s=-0.2, steer_angle_deg=0.0)

                            # ===== 阶段0.6：进入后退模式（dy过小且未后退过）=====
                            elif abs(dy) < 5 and abs(dx) > 10 and not backup_done:
                                backup_mode = True
                                print("🔄 dy过小({:.0f}), 进入后退模式".format(dy))
                                rover.drive(speed_rad_s=-0.2, steer_angle_deg=0.0)

                            # ===== 终极平滑追踪：P控制 + 蟹行 =====
                            elif abs(dx) > 10 or abs(dy) > 10:
                                angle = math.degrees(math.atan2(dx, -dy))
                                distance = math.sqrt(dx**2 + dy**2)
                                speed = distance * 0.002
                                
                                # 限速保护
                                if speed > 0.4: speed = 0.4
                                if speed < 0.15: speed = 0.15
                                
                                # dy过大 目标在下方（近）→ 后退避让；否则前进/蟹行
                                # 滞回：dy>15进入后退，dy<5恢复前进，中间保持当前模式
                                if dy > 15:
                                    reverse_mode = True
                                elif dy < 5:
                                    reverse_mode = False

                                if reverse_mode:
                                    speed = -speed
                                    angle = -math.degrees(math.atan2(dx, dy))

                                rover.drive(speed_rad_s=speed, steer_angle_deg=angle)
                                
                            # ===== 阶段3：抓取 =====
                            elif abs(dx) <= 10 and abs(dy) <= 10:
                                backup_done = False
                                reverse_mode = False
                                rover.stop()
                                rover.center_chassis_servos()
                                time.sleep_ms(300)
                                rover.stop()
                                print("✋ 目标已对准，开始抓取！")
                                
                                # 保存当前相机角度，张至最大（0°），为机械臂腾出空间
                                saved_cam_angle = 0.0
                                if rover.arm is not None:
                                    saved_cam_angle = rover.arm.camera_angle_deg
                                    try:
                                        rover.arm.jog_camera(-saved_cam_angle)
                                    except ArmKinematicsError as err:
                                        print_arm_error(err)
                                    time.sleep_ms(500)
                                
                                # 机械臂抓取放置（固定原始舵机角度，与 START 打印一致）
                                if rover.arm is not None:
                                    # 1) 过渡位姿 1：7=-19.9, 9=-0.1, 10=-140.9, 11=0
                                    print("🦾 运动到过渡位姿 1")
                                    _move_arm_servos(
                                        rover,
                                        ((7, -19.9), (9, -0.1), (10, -140.9), (11, 0.0)),
                                        speed_deg_s=_ARM_GRAB_SPEED,
                                    )
                                    # 2) 过渡位姿 2：7=-36.8, 9=-1.7, 10=-131.2, 11=0
                                    print("🦾 运动到过渡位姿 2")
                                    _move_arm_servos(
                                        rover,
                                        ((7, -36.8), (9, -1.7), (10, -131.2), (11, 0.0)),
                                        speed_deg_s=_ARM_GRAB_SPEED,
                                    )
                                    # 3) 运动到抓取姿态：7=-59.2, 9=0, 10=-116.7, 11=-5
                                    print("🦾 运动到抓取姿态")
                                    _move_arm_servos(
                                        rover,
                                        ((7, -59.2), (9, 0.0), (10, -116.7), (11, -5)),
                                        speed_deg_s=_ARM_GRAB_SPEED,
                                    )
                                    # 夹爪保持夹紧
                                    rover.servo_control.set_reserve_servo_angle(
                                        _GRIPPER_SERVO_ID, _GRIPPER_CLOSE_DEG, speed_deg_s=60.0
                                    )
                                    time.sleep_ms(_GRIPPER_SETTLE_MS)

                                    # 4) 放置前过渡位姿：7=-19.9, 9=-0.1, 10=-140.9, 11=0
                                    print("🦾 运动到放置前过渡位姿")
                                    _move_arm_servos(
                                        rover,
                                        ((7, -19.9), (9, -0.1), (10, -140.9), (11, 0.0)),
                                        speed_deg_s=_ARM_GRAB_SPEED,
                                    )

                                    # 5) 运动到放置姿态：7=-24.2, 9=0.0, 10=120.7, 11=0.4
                                    print("📦 运动到放置姿态")
                                    _move_arm_servos(
                                        rover,
                                        ((7, -24.2), (9, 0.0), (10, 120.7), (11, 0.4)),
                                        speed_deg_s=_ARM_GRAB_SPEED,
                                    )
                                    # 夹爪松开
                                    rover.servo_control.set_reserve_servo_angle(
                                        _GRIPPER_SERVO_ID, _GRIPPER_OPEN_DEG, speed_deg_s=60.0
                                    )
                                    time.sleep_ms(_GRIPPER_SETTLE_MS)
                                
                                # 机械臂复位
                                if rover.arm is not None:
                                    _pre_reset = dict(
                                        rover.servo_control.servo_bus.read_angles(_ARM_SERVO_IDS)
                                    )
                                    try:
                                        rover.arm.apply_initial_pose()
                                    except ArmKinematicsError as err:
                                        print_arm_error(err)
                                    # 初始位对应的原始舵机角度（与 robot_config 的 ARM_INIT_* 对应：
                                    # 7=PITCH1=50, 9=ROLL=0, 10=PITCH2=-140, 11=PITCH3=0）
                                    _reset_max_delta = 0.0
                                    for _sid, _tgt in ((7, 50.0), (9, 0.0), (10, -140.0), (11, 0.0)):
                                        _cur = _pre_reset.get(_sid)
                                        if _cur is not None:
                                            _reset_max_delta = max(_reset_max_delta, abs(_tgt - float(_cur)))
                                    time.sleep_ms(int(_reset_max_delta / _ARM_GRAB_SPEED * 1000) + 400)
                                    try:
                                        rover.servo_control.init_reserve_servos()
                                    except Exception as err:
                                        print("预留舵机复位失败：", err)
                                    time.sleep_ms(_GRIPPER_SETTLE_MS)
                                    
                                    # 相机恢复到追踪时的角度
                                    try:
                                        rover.arm.sync_camera_from_servo()
                                        delta = saved_cam_angle - rover.arm.camera_angle_deg
                                        print("📷 恢复相机: 当前{:.1f}° → 目标{:.1f}° (delta={:.1f}°)".format(
                                            rover.arm.camera_angle_deg, saved_cam_angle, delta))
                                        rover.arm.jog_camera(delta)
                                        time.sleep_ms(500)
                                    except ArmKinematicsError as err:
                                        print_arm_error(err)
                                
                                print("✅ 抓取放置完成！")
                                serial.write(b"ok\n")
                                
                                grab_cooldown = True
                                grab_cooldown_time = time.time()

                        except Exception as e:
                            pass
            
            # 👇 核心救命代码：强制每次循环休息 30 毫秒，防止手柄通讯崩溃！
            time.sleep_ms(30)
            
            # 只要是抓取模式，处理完就从头开始，跳过下面的手动控制
            continue
        
        # 【L2 键】按住时进入机械臂模式（放在R1之前，以便L2+R1控制夹爪）
        if button_pressed(buttons, ps2.PS2_BTN_L2):
            if not arm_mode_active:
                rover.stop()
                if not sync_arm_control_state(rover):
                    time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
                    continue
                arm_mode_active = True
            handle_arm_control(rover, ps2, buttons, lx, ly, rx, ry)
            time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
            continue

        # 【R1 键】紧急停止
        if button_pressed(buttons, ps2.PS2_BTN_R1):
            rover.stop()
            arm_mode_active = False
            time.sleep_ms(100)
            continue

        # 【CROSS 键】切换自动循迹模式（沿绿线行走）
        if button_pressed(buttons, ps2.PS2_BTN_CROSS):
            line_track_mode = not line_track_mode
            if line_track_mode:
                print("🟢 进入自动循迹模式（沿绿线行走）")
                rover.stop()
                grab_mode = False
                arm_mode_active = False
                line_track_lost = False
                line_track_lost_time = 0
                data["value"] = None  # 清除残留数据
                try:
                    serial.write(b"track_start\n")
                    print("[UART TX] 发送 track_start")
                except:
                    pass
            else:
                print("⏹️ 退出自动循迹模式")
                rover.stop()
                try:
                    serial.write(b"track_stop\n")
                    print("[UART TX] 发送 track_stop")
                except:
                    pass
            time.sleep_ms(300)  # 防抖
            continue

        # ==========================================================
        # 自动循迹模式的控制逻辑
        # ==========================================================
        if line_track_mode:
            serial_data = data["value"]
            if serial_data is not None:
                data["value"] = None
                parts = serial_data.strip().split()
                if len(parts) == 2:
                    try:
                        dx = int(parts[0])
                        angle = int(parts[1])
                        
                        # ===== 绿线丢失巡游 =====
                        if dx == 999:
                            if not line_track_lost:
                                line_track_lost = True
                                line_track_lost_time = time.time()
                                print("🔍 绿线丢失，原地缓慢搜索...")
                            
                            # 丢失超时自动退出循迹
                            if time.time() - line_track_lost_time > _LINE_TRACK_LOST_TIMEOUT:
                                print("⏰ 绿线丢失超时，自动退出循迹模式")
                                line_track_mode = False
                                rover.stop()
                                try:
                                    serial.write(b"track_stop\n")
                                except:
                                    pass
                                continue
                            
                            # 原地小角度左右搜索
                            rover.drive(speed_rad_s=0.0, steer_angle_deg=30.0)
                            time.sleep_ms(100)
                            rover.drive(speed_rad_s=0.0, steer_angle_deg=-30.0)
                            time.sleep_ms(100)
                        
                        # ===== 正常循迹 =====
                        else:
                            line_track_lost = False
                            
                            # 基于dx计算转向角
                            # dx>0: 线在右侧，需右转（正转向角）
                            # dx<0: 线在左侧，需左转（负转向角）
                            steer_angle = 0.0
                            if abs(dx) > _LINE_TRACK_DX_THRESHOLD:
                                # P控制：转向角与dx成正比
                                steer_ratio = dx / 320.0  # 归一化（图像半宽320）
                                steer_ratio = clamp(steer_ratio, -1.0, 1.0)
                                steer_angle = steer_ratio * _LINE_TRACK_MAX_STEER
                            
                            # 叠加线的角度信息（angle表示线的弯曲方向）
                            steer_angle += angle * 0.5
                            steer_angle = clamp(steer_angle, -_LINE_TRACK_MAX_STEER, _LINE_TRACK_MAX_STEER)
                            
                            # 速度调节：转弯越大速度越慢
                            speed = _LINE_TRACK_SPEED * (1.0 - abs(steer_angle) / _LINE_TRACK_MAX_STEER * 0.5)
                            if speed < 0.1:
                                speed = 0.1
                            
                            rover.drive(speed_rad_s=speed, steer_angle_deg=steer_angle)
                            
                            if abs(dx) > 50:
                                print(f"🟢 循迹: dx={dx:+d} angle={angle:+d} → 转向={steer_angle:+.1f}° 速度={speed:.2f}")
                    
                    except Exception as e:
                        pass
            
            time.sleep_ms(30)
            continue

        # 【TRIANGLE 键】电机使能
        if button_pressed(buttons, ps2.PS2_BTN_TRIANGLE):
            rover.enable_motors()
            arm_mode_active = False
            time.sleep_ms(200)
            continue

        arm_mode_active = False
        
        # 【R2 键】按住时，将右摇杆信息分配给底盘进行原地转向
        if button_pressed(buttons, ps2.PS2_BTN_R2):
            turn = map_joystick(rx)
            turn_speed = turn / 100.0 * _MAX_PIVOT_RAD_S
            rover.pivot_turn(turn_speed)
            time.sleep_ms(50)
            continue

        # 【常规底盘行驶】
        # ry (右摇杆Y轴) 作为油门
        throttle = -map_joystick(ry)
        # lx (左摇杆X轴) 作为转向舵
        steer = map_joystick(lx)
        
        speed_rad_s = throttle / 100.0 * _MAX_MOTOR_RAD_S
        steer_angle_deg = steer / 100.0 * MAX_STEER_ANGLE_DEG
        
        rover.drive(speed_rad_s, steer_angle_deg)
        time.sleep_ms(50)