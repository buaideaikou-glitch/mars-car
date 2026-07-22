from maix import image, uart, camera, display
from maix import camera, display, uart, app, time
import threading
cam = camera.Camera(320, 240)   # 屏幕大小
disp = display.Display()

ewm = []

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

while 1:
    img = cam.read()
    qrcodes = img.find_qrcodes()       # 寻找二维码，并将查询结果保存到qrocdes，找不到二维码则qrcodes内部为空
    for qr in qrcodes:                 # 如果识别到不为空
        corners = qr.corners()         # 用来获取已扫描到的二维码的四个顶点坐标
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_RED)  # 绘制画框
        img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)                    
        print(qr.payload())      
        ewm = qr.payload()                                                                                         # qr.payload()用来获取二维码的内容
        serial.write_str(ewm)  
    disp.show(img)
    
