from maix import camera, display, image, app, uart, time
import threading

xunhuan_num = 0
thresholds_red   = [[20, 60, 35, 80, 10, 40]]    # red
thresholds_blue = [[25, 70, -20, 15, -75, -25]]    # blue
thresholds_yellow = [[50, 100, -30,  10, 60, 100]]    # yellow
thresholds_pink = [[40, 70, 10, 40, -5, 10]]
thresholds_purple = [[-5, 55, 10, 50, -60, -10]]
# 串口初始化
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

def re_uart(serial):
    while 1:
        data = serial.read()
        if data:
            data = data.decode("utf-8", errors="ignore")
            print(f"uart0:{data}")
        time.sleep_ms(5)

uart0_thread = threading.Thread(target=re_uart, args=(serial,))
uart0_thread.daemon = True
uart0_thread.start()

#摄像头初始化
cam = camera.Camera(320, 240, fps=60 ) 
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    
    if xunhuan_num >= 1:  #60帧判断一次
        blobs = img.find_blobs(thresholds_red, pixels_threshold=10)  #pixels_threshold 色块阈值
        last_x_state=''
        last_y_state=''
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("red")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                offset_x=blob[5]-160
                offset_y=blob[6]-120
                #判断当前状态
                if offset_x>20:
                    current_x_state='向右移动'
                elif offset_x<-20:
                    current_x_state='向左移动'
                else:
                    current_x_state='已对准'

                if offset_y>20:
                    current_y_state='向下移动'
                elif offset_y<-20:
                    current_y_state='向上移动'
                else:
                    current_y_state='已对准'
                #触发输出
                if current_x_state!=last_x_state or current_y_state!=last_y_state:
                    print(f"{current_x_state} {current_y_state}")
                    last_x_state=current_x_state
                    last_y_state=current_y_state

                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_RED, 5)       # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽 ）

        blobs = img.find_blobs(thresholds_blue, pixels_threshold=10)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("blue")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_BLUE, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）


        blobs = img.find_blobs(thresholds_yellow, pixels_threshold=10)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("yellow")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_YELLOW, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）
        
        blobs = img.find_blobs(thresholds_pink, pixels_threshold=30)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("pink")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_WHITE, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）
        
        blobs = img.find_blobs(thresholds_purple, pixels_threshold=10)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("purple")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_BLACK, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）


        # 二维码识别
        qrcodes = img.find_qrcodes()
        for qr in qrcodes:
            corners = qr.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                              image.COLOR_RED)
            img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)
            print(qr.payload())
            serial.write_str(qr.payload())

        xunhuan_num = 0

    img.draw_rect( 320, 240, 30, 30, image.COLOR_RED, 5)   #在中心绘制中心点
    xunhuan_num = xunhuan_num + 1  
    disp.show(img)
