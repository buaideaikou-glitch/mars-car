from maix import camera, display, image, app, uart, time
import threading

xunhuan_num = 0

thresholds_red    = [[20, 60, 35, 80, 10, 40]]
thresholds_blue   = [[25, 70, -20, 15, -75, -25]]
thresholds_yellow = [[50, 100, -30, 10, 60, 100]]
thresholds_pink   = [[40, 70, 10, 40, -5, 10]]
thresholds_purple = [[-5, 55, 10, 50, -60, -10]]

ALL_COLORS = [
    ("red",    thresholds_red,    image.COLOR_RED,    10),
    ("blue",   thresholds_blue,   image.COLOR_BLUE,   10),
    ("yellow", thresholds_yellow, image.COLOR_YELLOW, 10),
    ("pink",   thresholds_pink,   image.COLOR_WHITE,  30),
    ("purple", thresholds_purple, image.COLOR_BLACK,  10),
]

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

cam = camera.Camera(320, 240, fps=60)
disp = display.Display()

while not app.need_exit():
    img = cam.read()

    if xunhuan_num >= 1:
        color_results = []
        all_cx_sum = 0.0
        all_cy_sum = 0.0
        all_pixels = 0

        for c_name, thresholds, draw_color, pix_th in ALL_COLORS:
            blobs = img.find_blobs(thresholds, pixels_threshold=pix_th)
            if blobs:
                count = len(blobs)
                cx_sum = 0.0
                cy_sum = 0.0
                for blob in blobs:
                    cx_sum += blob[5]
                    cy_sum += blob[6]
                    all_pixels += blob[2] * blob[3]
                    img.draw_rect(blob[0], blob[1], blob[2], blob[3],
                                  draw_color, 3)

                avg_cx = cx_sum / count
                avg_cy = cy_sum / count
                color_results.append((c_name, count, avg_cx, avg_cy))
                all_cx_sum += cx_sum
                all_cy_sum += cy_sum

        color_results.sort(key=lambda x: x[2])

        total_count = sum(r[1] for r in color_results)
        overall_cx = all_cx_sum / total_count if total_count > 0 else 160.0
        overall_cy = all_cy_sum / total_count if total_count > 0 else 120.0

        top3 = sorted(color_results, key=lambda x: -x[1])[:3]
        c1, n1 = top3[0][0], top3[0][1] if len(top3) > 0 else ("none", 0)
        c2, n2 = top3[1][0], top3[1][1] if len(top3) > 1 else ("none", 0)
        c3, n3 = top3[2][0], top3[2][1] if len(top3) > 2 else ("none", 0)

        qrcodes = img.find_qrcodes()
        qr_payload = "NONE"
        if qrcodes:
            for qr in qrcodes:
                corners = qr.corners()
                for i in range(4):
                    img.draw_line(corners[i][0], corners[i][1],
                                  corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                                  image.COLOR_RED)
                img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)
                qr_payload = qr.payload().replace(" ", "_")
                break

        msg = "{} {} {} {} {} {} {:.1f} {:.1f} {} {}".format(
            c1, n1, c2, n2, c3, n3,
            overall_cx, overall_cy, int(all_pixels), qr_payload)
        serial.write_str(msg + "\n")
        print("sent:", msg)

        xunhuan_num = 0

    img.draw_rect(155, 115, 10, 10, image.COLOR_GREEN, 2)
    xunhuan_num += 1
    disp.show(img)
