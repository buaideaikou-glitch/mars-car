from maix import camera, display, image, app
from maix import uart, time

xunhuan_num = 0
thresholds_red   = [[0, 80,  40,  80,   10,  80]]    # red
thresholds_green = [[0, 60,   0, 127, -128, -35]]    # green
thresholds_black = [[9, 29, -10,  10,  -11,   9]]    # black

#摄像头初始化
cam = camera.Camera(640, 480, fps=60 ) 
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    
    if xunhuan_num >= 1 :  #60帧判断一次
        blobs = img.find_blobs(thresholds_red, pixels_threshold=10)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("red")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_RED, 5)       # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽 ）

        blobs = img.find_blobs(thresholds_green, pixels_threshold=10)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("blue")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_GREEN, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）


        blobs = img.find_blobs(thresholds_black, pixels_threshold=1000)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            for blob in blobs:
                print("black")
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_BLACK, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）
        xunhuan_num = 0

    img.draw_rect( 320, 240, 30, 30, image.COLOR_RED, 5)   #在中心绘制中心点
    xunhuan_num = xunhuan_num + 1  
    disp.show(img)

  



