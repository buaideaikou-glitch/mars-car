"""
月球小车底盘组合控制（非底层协议库）。

依赖 motor_lib.MotorBus 与 servo_control.ServoControl，负责把电机和转向舵机
组合成整车运动接口。底层 CAN/UART 协议见 motor_lib.py、servo_lib.py。

常用学生接口：
- prepare()
- enable_motors()
- disable()
- stop()
- drive(speed_rad_s, steer_angle_deg)
- pivot_turn(speed_rad_s)

作者 王笑
日期 20260528
"""

import math
import time

from robot_config import (
    DEFAULT_ACC_RAD_S2,
    MAX_MOTOR_RPM,
    MAX_STEER_ANGLE_DEG,
    PIVOT_SPEED_SCALE,
    clamp,
)

_HALF_WHEELBASE_MM = 200.0
_HALF_TRACK_MM = 150.0
_STEER_SIGN = -1
_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_PIVOT_STEER_ANGLE_DEG = math.degrees(math.atan(_HALF_WHEELBASE_MM / _HALF_TRACK_MM))
_STEER_SERVO_SPEED_DEG_S = 180.0
_STEER_CMD_EPS_DEG = 0.1
_MOTOR_CMD_EPS_RAD_S = 0.01
_ACC_CMD_EPS_RAD_S2 = 0.01

_DRIVE_WHEELS = (
    {"name": "left_front", "motor_id": 1, "x": _HALF_WHEELBASE_MM,
     "y": _HALF_TRACK_MM, "direction": -1},
    {"name": "right_front", "motor_id": 2, "x": _HALF_WHEELBASE_MM,
     "y": -_HALF_TRACK_MM, "direction": 1},
    {"name": "left_rear", "motor_id": 3, "x": -_HALF_WHEELBASE_MM,
     "y": _HALF_TRACK_MM, "direction": -1},
    {"name": "right_rear", "motor_id": 4, "x": -_HALF_WHEELBASE_MM,
     "y": -_HALF_TRACK_MM, "direction": 1},
)


class LunarRover:
    def __init__(self, motor_bus, servo_control, arm=None):
        self.motor_bus = motor_bus
        self.servo_control = servo_control
        self.servo_bus = servo_control.servo_bus
        self.arm = arm
        self.motor_ids = tuple(w["motor_id"] for w in _DRIVE_WHEELS)
        self.motors_enabled = False
        self._motors_stopped = True
        self._last_steering_angles = None
        self._last_steering_speed_deg_s = None
        self._last_motor_acc = {}
        self._last_motor_speeds = {}

    def _reset_command_cache(self):
        self._last_steering_angles = None
        self._last_steering_speed_deg_s = None
        self._last_motor_acc = {}
        self._last_motor_speeds = {}

    @staticmethod
    def _same_float(a, b, eps):
        return b is not None and abs(float(a) - float(b)) <= eps

    @staticmethod
    def _same_tuple(values, old_values, eps):
        if old_values is None or len(values) != len(old_values):
            return False
        for value, old_value in zip(values, old_values):
            if abs(float(value) - float(old_value)) > eps:
                return False
        return True

    def _set_steering_angles_cached(self, angles, speed_deg_s):
        angles = tuple(float(angle) for angle in angles)
        speed_deg_s = float(speed_deg_s)
        if (self._same_tuple(angles, self._last_steering_angles, _STEER_CMD_EPS_DEG)
                and self._same_float(speed_deg_s, self._last_steering_speed_deg_s, 0.01)):
            return False

        self.servo_control.set_steering_angles(
            angles[0], angles[1], angles[2],
            angles[3], angles[4], angles[5],
            speed_deg_s=speed_deg_s,
        )
        self._last_steering_angles = angles
        self._last_steering_speed_deg_s = speed_deg_s
        return True

    def _set_motor_acc_cached(self, motor_id, acc_rad_s2):
        old_acc = self._last_motor_acc.get(motor_id)
        if self._same_float(acc_rad_s2, old_acc, _ACC_CMD_EPS_RAD_S2):
            return False
        self.motor_bus.set_acc(motor_id, acc_rad_s2)
        self._last_motor_acc[motor_id] = float(acc_rad_s2)
        return True

    def _set_motor_speed_cached(self, motor_id, speed_rad_s):
        old_speed = self._last_motor_speeds.get(motor_id)
        if self._same_float(speed_rad_s, old_speed, _MOTOR_CMD_EPS_RAD_S):
            return False
        self.motor_bus.set_speed(motor_id, speed_rad_s)
        self._last_motor_speeds[motor_id] = float(speed_rad_s)
        return True

    def prepare(self):
        self.center_chassis_servos()
        if self.arm is not None:
           # self.arm.apply_initial_pose()
            time.sleep_ms(120)
        self.enable_motors()

    def enable_motors(self):
        """使能四轮：init_speed_mode 内先失能再写速度模式与 PI，最后使能。"""
        self.motor_bus.prepare_speed_mode(self.motor_ids)
        self.motors_enabled = True
        self._motors_stopped = True
        self._reset_command_cache()

    def stop(self):
        if self._motors_stopped:
            return
        self.motor_bus.stop_all(self.motor_ids)
        for motor_id in self.motor_ids:
            self._last_motor_speeds[motor_id] = 0.0
        self._motors_stopped = True

    def disable(self):
        self.motor_bus.disable_all(self.motor_ids)
        self.motors_enabled = False
        self._motors_stopped = True
        self._reset_command_cache()

    def center_chassis_servos(self, speed_deg_s=_STEER_SERVO_SPEED_DEG_S):
        sent = self._set_steering_angles_cached(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            speed_deg_s,
        )
        if sent:
            time.sleep_ms(20)

    def steer_only(self, angle_deg, speed_deg_s=_STEER_SERVO_SPEED_DEG_S):
        sent = self._set_steering_angles_cached(
            (angle_deg, angle_deg, angle_deg, angle_deg, angle_deg, angle_deg),
            speed_deg_s,
        )
        if sent:
            time.sleep_ms(20)

    def drive(self, speed_rad_s, steer_angle_deg,
              steer_speed_deg_s=_STEER_SERVO_SPEED_DEG_S):
        """
        speed_rad_s：期望车轮角速度，单位 rad/s，正数前进，负数后退。
        steer_angle_deg：期望底盘转向角，单位 deg。
        """
        motor_speed = clamp(float(speed_rad_s), -_MAX_MOTOR_RAD_S, _MAX_MOTOR_RAD_S)
        steer_angle = clamp(
            float(steer_angle_deg),
            -MAX_STEER_ANGLE_DEG,
            MAX_STEER_ANGLE_DEG,
        ) * _STEER_SIGN

        self._set_steering_angles_cached(
            (
                steer_angle, steer_angle, steer_angle,
                steer_angle, steer_angle, steer_angle,
            ),
            steer_speed_deg_s,
        )

        for wheel in _DRIVE_WHEELS:
            motor_id = wheel["motor_id"]
            speed = motor_speed * wheel.get("direction", 1)
            self._set_motor_acc_cached(motor_id, DEFAULT_ACC_RAD_S2)
            self._set_motor_speed_cached(motor_id, speed)
        self._motors_stopped = abs(motor_speed) < 0.01

    def pivot_turn(self, speed_rad_s,
                   steer_speed_deg_s=_STEER_SERVO_SPEED_DEG_S):
        """
        原地转向：speed_rad_s 为原地转向轮速，负数左转，正数右转。
        """
        max_pivot_speed = _MAX_MOTOR_RAD_S * PIVOT_SPEED_SCALE
        speed_rad_s = clamp(float(speed_rad_s), -max_pivot_speed, max_pivot_speed)
        if abs(speed_rad_s) < 0.01:
            self.center_chassis_servos(
                speed_deg_s=steer_speed_deg_s,
            )
            self.stop()
            return

        abs_speed = abs(speed_rad_s)
        if speed_rad_s < 0:
            left_cmd = -abs_speed
            right_cmd = abs_speed
        else:
            left_cmd = abs_speed
            right_cmd = -abs_speed

        left_front_angle = _PIVOT_STEER_ANGLE_DEG * _STEER_SIGN
        left_rear_angle = -_PIVOT_STEER_ANGLE_DEG * _STEER_SIGN
        right_front_angle = -_PIVOT_STEER_ANGLE_DEG * _STEER_SIGN
        right_rear_angle = _PIVOT_STEER_ANGLE_DEG * _STEER_SIGN
        self._set_steering_angles_cached(
            (
                left_front_angle, 0.0, left_rear_angle,
                right_front_angle, 0.0, right_rear_angle,
            ),
            steer_speed_deg_s,
        )

        for wheel in _DRIVE_WHEELS:
            motor_id = wheel["motor_id"]
            side_speed = left_cmd if wheel["y"] > 0 else right_cmd
            speed = side_speed * wheel.get("direction", 1)
            self._set_motor_acc_cached(motor_id, DEFAULT_ACC_RAD_S2)
            self._set_motor_speed_cached(motor_id, speed)
        self._motors_stopped = False
