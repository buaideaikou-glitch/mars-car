"""
月球小车示例主程序。

底层协议已经拆成库：
- motor_lib.py：电机 CAN 速度模式
- servo_lib.py：Fashion Star 舵机二进制协议
- servo_control.py：按功能划分的学生舵机控制接口
- chassis_control.py：底盘组合控制（依赖 motor_lib + servo_control）
- arm_control.py：机械臂关节点动与相机控制（依赖 servo_control）
- robot_config.py：ID、引脚、速度、尺寸等配置

作者 王笑
日期 20260528
"""
import time, _thread, sys, machine
time.sleep(3)

from machine import UART
from esp32 import CAN
from ps2_lib import PS2Controller, PS2Receiver
from ps2_control1 import ps2_loop


from arm_control import RobotArm
from motor_lib import MotorBus
from chassis_control import LunarRover
from servo_control import ServoControl, get_all_servo_ids
from servo_lib import ServoBus
from robot_config import (
    CAN_BAUDRATE,
    CAN_BUS_ID,
    CAN_RX,
    CAN_TX,
    CAMERA_SERVO_ID,
    DEFAULT_ACC_RAD_S2,
    PS2_CLK,
    PS2_CS,
    PS2_DI,
    PS2_DO,
    RUN_MODE,
    SERVO_UART_BAUD,
    SERVO_UART_ID,
    SERVO_UART_RX,
    SERVO_UART_TX,
    CAMERA_UART_ID,
    CAMERA_UART_BAUD,
    CAMERA_UART_TX,
    CAMERA_UART_RX,
    RESERVE_SERVO_ENABLED,
    RESERVE_SERVO_IDS,
)

#定义变量
camera_data          = {"value": None}

#硬件配置与初始化
servo_uart = UART(
    SERVO_UART_ID,
    SERVO_UART_BAUD,
    tx=SERVO_UART_TX,
    rx=SERVO_UART_RX,
    timeout=64,
)

camera_uart = UART(
    CAMERA_UART_ID,
    CAMERA_UART_BAUD,
    tx=CAMERA_UART_TX,
    rx=CAMERA_UART_RX,
    timeout=64
)

try:
    can = CAN(
        CAN_BUS_ID,
        mode=CAN.NORMAL,
        baudrate=CAN_BAUDRATE,
        tx=CAN_TX,
        rx=CAN_RX,
    )
except Exception as e:
    print(f"CAN硬件占用。触发系统级软复位，别慌，请点击STOP重新连接")
    time.sleep(1)
    machine.reset()
can.clear_rx_queue()

#多线程通讯函数
def re_uart(uart):
    global camera_data, camera_uart
    print("uart启动")
    loop_count = 0
    try:
        while True:
            loop_count += 1
            has_data = uart.any()
            if loop_count % 20 == 0:
                pass
                #print(f"⏳ 等待数据... uart.any() = {has_data}")
            if uart.any() and uart == camera_uart :
                data= uart.readline()
                if data:
                    camera_data["value"] = data.decode("utf-8", "replace").strip()
                    #print("串口1收到数据:", camera_data)
            time.sleep_ms(10)       # 防止形成阻塞              
    except UnicodeError:
        print("【成功拦截乱码】串口1收到一串无法识别的非文本数据:")
        pass
        _thread.start_new_thread(re_uart, (uart, ))


motor_bus = MotorBus(can)
servo_bus = ServoBus(servo_uart)
servo_bus.reset_turns_polling(get_all_servo_ids()) #清除多圈
servo_bus.lock_all(get_all_servo_ids())   #舵机锁力
servo_control = ServoControl(servo_bus)
servo_control.init_reserve_servos()
arm = RobotArm(servo_control) #基于运动学的舵机控制接口
rover = LunarRover(motor_bus, servo_control, arm=arm)

#打开多线程
_thread.start_new_thread(re_uart, (camera_uart, ))

def main():
    global camera_data
    try:    
        if RUN_MODE == "ps2":
            rover.prepare()
            ps2_controller = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
            ps2_controller.init_vibration()
            ps2 = PS2Receiver(ps2_controller, 30, True)
            ps2.start()
            try:
                ps2_loop(rover, ps2, camera_data, camera_uart)
            finally:
                ps2.stop()
                rover.disable()
            return

        rover.prepare() #初始化底盘 电机使能
        print("RUN_MODE=idle，电机已默认使能。学生可在示例区编写一次性控制程序。")

        # ================= 学生控制示例 =================
        # 使用方法：每次只取消一小段示例代码的注释，确认安全后再运行。
        # 注意：调试底盘前建议先架空车轮，避免小车突然运动。

        # 示例 1：底盘以 2.0 rad/s 前进 1 秒，然后停车。
        print("示例 1：底盘以 2.0 rad/s 前进 1 秒，然后停车。")
        rover.drive(speed_rad_s=2.0, steer_angle_deg=0.0)
        time.sleep(1) #必须要延迟，让指令有执行时间，如果没有sleep，指令会被立即覆盖，导致小车没有动作。
        rover.stop()
        time.sleep(1)

        # # 示例 2：底盘以 2.0 rad/s、20 度转向角前进 1 秒，然后停车。
        # print("示例 2：底盘以 2.0 rad/s、20 度转向角前进 1 秒，然后停车。")
        # rover.drive(speed_rad_s=2.0, steer_angle_deg=20.0)
        # time.sleep(1)
        # rover.stop()
        # time.sleep(1)

        # # 示例 3：相机转到 -30 度，再回到 0 度。
        # print("示例 3：相机转到 -30 度，再回到 0 度。")
        # rover.servo_control.set_camera_angle(-30)
        # time.sleep(1)
        # rover.servo_control.set_camera_angle(0)
        # time.sleep(1)

        # # 示例 4：机械臂回初始位。
        # print("示例 4：机械臂回初始位。")
        # rover.arm.apply_initial_pose()
        # time.sleep(2) #要给机械臂足够时间回初始位，否则后续指令可能会被覆盖，导致机械臂无法到达目标位置。

        # # 示例 5：机械臂单关节点动，分别控制 Roll、Pitch1、Pitch2、Pitch3。
        # # 如果前面运行过绝对角度控制，先同步真实舵机角度，再做 jog 增量控制。
        # print("示例 5：机械臂单关节点动，分别控制 Roll、Pitch1、Pitch2、Pitch3。")
        # rover.arm.sync_from_servos()
        # rover.arm.jog_joints(roll_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(roll_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch1_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch1_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch2_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch2_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch3_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch3_delta_deg=-2.0)
        # time.sleep(1)

        # # 示例 6：相机舵机点动，再转回。
        # # 如果前面运行过相机绝对角度控制，先同步真实相机角度，再做 jog 增量控制。
        # print("示例 6：相机舵机点动，再转回。")
        # rover.arm.sync_camera_from_servo() #使用jog控制前，要先同步一次当前舵机的角度
        # rover.arm.jog_camera(8.0)
        # time.sleep(1)
        # rover.arm.jog_camera(-8.0)
        # time.sleep(1)

        # # 示例 7：四个驱动电机分别设置不同转速，运行 1 秒后停车。
        # # 注意：这是直接控制电机，不会自动调整转向舵机角度。
        # print("示例 7：四个驱动电机分别设置不同转速，运行 1 秒后停车。")
        # motor_speeds = (
        #     (1, 1.0),    # 左前电机 ID=1，速度 1.0 rad/s
        #     (2, 0.5),    # 右前电机 ID=2，速度 0.5 rad/s
        #     (3, -0.5),   # 左后电机 ID=3，速度 -0.5 rad/s
        #     (4, -1.0),   # 右后电机 ID=4，速度 -1.0 rad/s
        # )
        # for motor_id, speed_rad_s in motor_speeds:
        #     rover.motor_bus.set_acc(motor_id, DEFAULT_ACC_RAD_S2)
        #     rover.motor_bus.set_speed(motor_id, speed_rad_s)
        # time.sleep(1)
        # rover.motor_bus.stop_all(rover.motor_ids)
        # time.sleep(1)

        # # 示例 8：六个转向舵机分别设置不同角度。
        # # 参数顺序：左前、左中、左后、右前、右中、右后。
        # # print("示例 8：六个转向舵机分别设置不同角度。")
        # rover.servo_control.set_steering_angles(
        #     20.0, 0.0, -20.0,
        #     -20.0, 0.0, 20.0,
        # )
        # time.sleep(1)
        # rover.center_chassis_servos()
        # time.sleep(1)

        # # 示例 9：直接设置机械臂四个关节的目标角度。
        # # 参数顺序：Roll、Pitch1、Pitch2、Pitch3。
        # # 这是直接舵机角度控制，适合做固定姿态演示。
        # print("示例 9：直接设置机械臂四个关节的目标角度。")
        # rover.servo_control.set_arm_joint_angles(
        #     0.0, 50.0, -140.0, 0.0,
        # )
        # time.sleep(1)

        # # 示例 10：读取机械臂四个关节角度。
        # print("示例 10：读取机械臂四个关节角度。")
        # print(rover.servo_control.read_arm_joint_angles())
        # time.sleep(1)

        # # 示例 11：预留舵机测试。
        # # 需先在 robot_config.py 设置 RESERVE_SERVO_ENABLED = True，并在 RESERVE_SERVO_IDS 中填写 ID。
        # print("示例 11：预留舵机测试。")
        # if not RESERVE_SERVO_ENABLED:
        #     print("预留舵机未启用，请先在 robot_config.py 设置 RESERVE_SERVO_ENABLED = True。")
        # else:
        #     reserve_id = RESERVE_SERVO_IDS[0]
        #     rover.servo_control.set_reserve_servo_angle(reserve_id, 30.0)
        #     time.sleep(1)
        #     angle = rover.servo_control.read_reserve_servo_angle(reserve_id)
        #     print("预留舵机 ID=%d 当前角度：" % reserve_id, angle)
        #     rover.servo_control.set_reserve_servo_angle(reserve_id, -2.0)
        #     time.sleep(1)
    except Exception as e:
        print( "错误代码：",e )
        
    while True:
        #print("正常运行")
        time.sleep_ms(500)


if __name__ == "__main__":
    main()

