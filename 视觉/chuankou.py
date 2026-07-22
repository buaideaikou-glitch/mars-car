
from maix import camera, display, uart, app, time
import threading

stuts  = ""

#摄像头初始化
cam  = camera.Camera(320, 240)
disp = display.Display()

# 串口初始化
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

def re_uart (serial) :
    global stuts
    while 1:
        # 串口  接收数据
        data = serial.read() 
        data = data.decode("utf-8",errors="ignore")
        if data != "" and serial == serial:  #串口0 赋值
            print(f"uart0:{data}")
            stuts  = data
            data    = ""

uart0_thread = threading.Thread(target=re_uart, args = (serial,))
uart0_thread.daemon = True
uart0_thread.start()

while not app.need_exit():
    img = cam.read()
    disp.show(img)
    time.sleep(1)
    u_id   = "sx"
    x_zb   = int(30.5)
    y_zb   = int(31)
    z_zb   = int(-20.12345131)
    u_data = f"{u_id}{x_zb:04d}{y_zb:04d}{z_zb:04d}".encode("utf-8")
    serial.write_str("123123")

    print(time.time())
    if stuts != "" :
        serial.write_str(u_data) 
        stuts = ""



