"""
学生舵机功能控制接口。

按功能划分的常用接口：
- set_steering_angles(left_front, left_middle, left_rear, right_front, right_middle, right_rear, speed_deg_s=180)
- set_camera_angle(camera_angle, speed_deg_s=60)
- set_arm_joint_angles(roll, pitch1, pitch2, pitch3, speed_deg_s=60)
- init_reserve_servos() / init_reserve_servo(servo_id, ...)
- set_reserve_servo_angle(servo_id, angle, speed_deg_s=60) / read_reserve_servo_angle(servo_id)

所有入口角度都会先按 robot_config.py 中的限位裁剪，再下发到底层总线。

作者 王笑
日期 20260528
"""

from robot_config import (
    ARM_PITCH1_MAX_DEG,
    ARM_PITCH1_MIN_DEG,
    ARM_PITCH2_MAX_DEG,
    ARM_PITCH2_MIN_DEG,
    ARM_PITCH3_MAX_DEG,
    ARM_PITCH3_MIN_DEG,
    ARM_ROLL_MAX_DEG,
    ARM_ROLL_MIN_DEG,
    ARM_SERVO_SPEED_DEG_S,
    BASE_SERVO_IDS,
    CAMERA_ANGLE_MAX_DEG,
    CAMERA_ANGLE_MIN_DEG,
    CAMERA_SERVO_ID,
    RESERVE_SERVO_ENABLED,
    RESERVE_SERVO_IDS,
    RESERVE_SERVO_SIGNS,
    RESERVE_SERVO_INIT_ANGLE_DEG,
    RESERVE_SERVO_MAX_DEG,
    RESERVE_SERVO_MIN_DEG,
    STEER_ANGLE_MAX_DEG,
    STEER_ANGLE_MIN_DEG,
    clamp,
)

_STEER_SERVO_SPEED_DEG_S = 180.0
_CAMERA_SERVO_SPEED_DEG_S = 60.0

_STEERING_SERVO_IDS = {
    "left_front": 1,
    "right_front": 2,
    "left_rear": 3,
    "right_rear": 4,
    "left_middle": 5,
    "right_middle": 6,
}

_ARM_PITCH1_SERVO_ID = 7
_ARM_ROLL_SERVO_ID = 9
_ARM_PITCH2_SERVO_ID = 10
_ARM_PITCH3_SERVO_ID = 11

_CAMERA_SIGN = -1
_ARM_ROLL_SIGN = -1
_ARM_PITCH1_SIGN = 1
_ARM_PITCH2_SIGN = 1
_ARM_PITCH3_SIGN = 1


def get_all_servo_ids():
    """返回上电初始化需要处理的全部舵机 ID。"""
    if RESERVE_SERVO_ENABLED:
        return BASE_SERVO_IDS + tuple(RESERVE_SERVO_IDS)
    return BASE_SERVO_IDS


def _servo_id_by_name(name):
    try:
        return _STEERING_SERVO_IDS[name]
    except KeyError:
        raise ValueError("unknown servo name: %s" % name)


def _safe_sign(sign):
    return 1 if sign == 0 else sign


def _reserve_sign(servo_id):
    return _reserve_value(servo_id, RESERVE_SERVO_SIGNS, "RESERVE_SERVO_SIGNS")


def _reserve_init_angle(servo_id):
    return _reserve_value(
        servo_id,
        RESERVE_SERVO_INIT_ANGLE_DEG,
        "RESERVE_SERVO_INIT_ANGLE_DEG",
    )


def _reserve_value(servo_id, table, name):
    try:
        return table[int(servo_id)]
    except KeyError:
        raise ValueError("servo_id %s missing in %s" % (servo_id, name))


class ServoControl:
    def __init__(self, servo_bus):
        self.servo_bus = servo_bus

        self.left_front_id = _servo_id_by_name("left_front")
        self.left_middle_id = _servo_id_by_name("left_middle")
        self.left_rear_id = _servo_id_by_name("left_rear")
        self.right_front_id = _servo_id_by_name("right_front")
        self.right_middle_id = _servo_id_by_name("right_middle")
        self.right_rear_id = _servo_id_by_name("right_rear")

    def init_reserve_servos(self):
        """
        初始化全部预留舵机：仅在 RESERVE_SERVO_ENABLED=True 时生效。
        将 RESERVE_SERVO_IDS 中的每个舵机转到各自配置的初始角。
        """
        if not RESERVE_SERVO_ENABLED:
            return False
        for servo_id in RESERVE_SERVO_IDS:
            self.init_reserve_servo(servo_id)
        return True

    def init_reserve_servo(self, servo_id, angle_deg=None):
        """
        初始化指定 ID 的预留舵机。
        angle_deg 省略时使用该 ID 在 RESERVE_SERVO_INIT_ANGLE_DEG 中的配置。
        """
        if not RESERVE_SERVO_ENABLED:
            return False
        if angle_deg is None:
            angle_deg = _reserve_init_angle(servo_id)
        return self.set_reserve_servo_angle(servo_id, angle_deg)

    def set_reserve_servo_angle(self, servo_id, angle_deg,
                                speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        """控制指定 ID 的预留舵机角度，使用速度型位置控制。"""
        if not RESERVE_SERVO_ENABLED:
            return False
        servo_id = int(servo_id)
        angle_deg = self._limit_reserve(servo_id, angle_deg)
        sign = _reserve_sign(servo_id)
        self.servo_bus.set_angles(
            ((servo_id, angle_deg * sign),),
            speed_deg_s=speed_deg_s,
        )
        return True

    def read_reserve_servo_angle(self, servo_id):
        """轮询读取指定 ID 的预留舵机角度；未启用时返回 None。"""
        if not RESERVE_SERVO_ENABLED:
            return None
        servo_id = int(servo_id)
        angle = self.servo_bus.read_angle(servo_id)
        if angle is None:
            return None
        return self._remove_sign(angle, _reserve_sign(servo_id))

    def set_steering_angles(self, left_front, left_middle, left_rear,
                            right_front, right_middle, right_rear,
                            speed_deg_s=_STEER_SERVO_SPEED_DEG_S):
        """控制 6 个底盘转向舵机，默认 180 deg/s。"""
        targets = (
            (self.left_front_id, self._limit_steer(left_front)),
            (self.left_middle_id, self._limit_steer(left_middle)),
            (self.left_rear_id, self._limit_steer(left_rear)),
            (self.right_front_id, self._limit_steer(right_front)),
            (self.right_middle_id, self._limit_steer(right_middle)),
            (self.right_rear_id, self._limit_steer(right_rear)),
        )
        self.servo_bus.set_angles(
            targets,
            speed_deg_s=speed_deg_s,
        )

    def set_camera_angle(self, camera_angle,
                         speed_deg_s=_CAMERA_SERVO_SPEED_DEG_S):
        """控制相机舵机角度，默认 60 deg/s。"""
        angle = clamp(float(camera_angle), CAMERA_ANGLE_MIN_DEG, CAMERA_ANGLE_MAX_DEG)
        self.servo_bus.set_angles(
            ((CAMERA_SERVO_ID, angle * _CAMERA_SIGN),),
            speed_deg_s=speed_deg_s,
        )

    def set_arm_joint_angles(self, roll, pitch1, pitch2, pitch3,
                             speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        """控制机械臂 Roll、Pitch1、Pitch2、Pitch3 四个关节，默认 60 deg/s。"""
        roll = clamp(float(roll), ARM_ROLL_MIN_DEG, ARM_ROLL_MAX_DEG)
        pitch1 = clamp(float(pitch1), ARM_PITCH1_MIN_DEG, ARM_PITCH1_MAX_DEG)
        pitch2 = clamp(float(pitch2), ARM_PITCH2_MIN_DEG, ARM_PITCH2_MAX_DEG)
        pitch3 = clamp(float(pitch3), ARM_PITCH3_MIN_DEG, ARM_PITCH3_MAX_DEG)

        self.servo_bus.set_angles(
            (
                (_ARM_ROLL_SERVO_ID, roll * _ARM_ROLL_SIGN),
                (_ARM_PITCH1_SERVO_ID, pitch1 * _ARM_PITCH1_SIGN),
                (_ARM_PITCH2_SERVO_ID, pitch2 * _ARM_PITCH2_SIGN),
                (_ARM_PITCH3_SERVO_ID, pitch3 * _ARM_PITCH3_SIGN),
            ),
            speed_deg_s=speed_deg_s,
        )

    def read_steering_angles(self):
        """轮询读取 6 个转向舵机角度，返回字典。"""
        ids = (
            self.left_front_id,
            self.left_middle_id,
            self.left_rear_id,
            self.right_front_id,
            self.right_middle_id,
            self.right_rear_id,
        )
        values = dict(self.servo_bus.read_angles(ids))
        return {
            "left_front": values.get(self.left_front_id),
            "left_middle": values.get(self.left_middle_id),
            "left_rear": values.get(self.left_rear_id),
            "right_front": values.get(self.right_front_id),
            "right_middle": values.get(self.right_middle_id),
            "right_rear": values.get(self.right_rear_id),
        }

    def read_camera_angle(self):
        """轮询读取相机舵机角度。"""
        angle = self.servo_bus.read_angle(CAMERA_SERVO_ID)
        return self._remove_sign(angle, _CAMERA_SIGN)

    def read_arm_joint_angles(self):
        """轮询读取机械臂关节角度，返回字典。"""
        servo_values = dict(self.servo_bus.read_angles(
            (
                _ARM_ROLL_SERVO_ID,
                _ARM_PITCH1_SERVO_ID,
                _ARM_PITCH2_SERVO_ID,
                _ARM_PITCH3_SERVO_ID,
            )
        ))
        return {
            "roll": self._remove_sign(servo_values.get(_ARM_ROLL_SERVO_ID), _ARM_ROLL_SIGN),
            "pitch1": self._remove_sign(servo_values.get(_ARM_PITCH1_SERVO_ID), _ARM_PITCH1_SIGN),
            "pitch2": self._remove_sign(servo_values.get(_ARM_PITCH2_SERVO_ID), _ARM_PITCH2_SIGN),
            "pitch3": self._remove_sign(servo_values.get(_ARM_PITCH3_SERVO_ID), _ARM_PITCH3_SIGN),
        }

    @staticmethod
    def _limit_steer(angle):
        return clamp(float(angle), STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG)

    @staticmethod
    def _limit_reserve(servo_id, angle):
        min_deg = _reserve_value(
            servo_id,
            RESERVE_SERVO_MIN_DEG,
            "RESERVE_SERVO_MIN_DEG",
        )
        max_deg = _reserve_value(
            servo_id,
            RESERVE_SERVO_MAX_DEG,
            "RESERVE_SERVO_MAX_DEG",
        )
        return clamp(float(angle), min_deg, max_deg)

    @staticmethod
    def _remove_sign(value, sign):
        if value is None:
            return None
        return float(value) / _safe_sign(sign)
