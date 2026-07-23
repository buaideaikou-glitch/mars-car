from maix import camera, display, image, uart, app, time
import threading
import re
import math

# ============ 全局变量 ============
stuts = ""
task_parsed = False
task_queue = []
current_color = None
remaining_count = 0
grab_count = 0
last_send_time = 0
send_interval = 0.2
center_x, center_y = 320, 240
send_success = False
send_success_time = 0
send_data = ""
locked_block = None
system_active = False
line_track_mode = False  # 自动循迹模式标志

# 对准容差
ALIGN_THRESHOLD = 5
ALIGN_COUNT_NEEDED = 10
align_count = 0

# 抓取超时：无ok回传时默认成功进入下一块（秒）
GRAB_TIMEOUT_SEC = 25.0
ok_wait_start = 0.0

# 已抓物块位置记录，防止重复抓取同一块
grabbed_positions = []
grabbed_block_pos = None

# 抓取状态
waiting_ok = False            # 对准后等待小车ok
car_grab_ok = False           # 收到ok

# 锁定丢失容忍度
LOCK_LOST_TOLERANCE = 10
lock_lost_count = 0

# ============ 色块识别阈值 ============
thresholds = {  
    "red":    [[20, 60, 35, 80, 10, 40]],
    "blue":   [[25, 70, -20, 15, -75, -25]],
    "yellow": [[50, 100, -30, 10, 60, 100]],
    "pink":   [[60, 80, 5, 20, -40, -10]],
    "purple": [[5, 30, 20, 45, -70, -35]]
}

color_draw = {
    "red": image.COLOR_RED,
    "blue": image.COLOR_BLUE,
    "yellow": image.COLOR_YELLOW,
    "purple": image.COLOR_PURPLE,
    "pink": image.COLOR_BLACK
}

# ============ 绿色线LAB阈值（待填写） ============
# TODO: 请在此处填写绿色线的LAB颜色阈值
# 格式: [[L_min, L_max, A_min, A_max, B_min, B_max]]
green_line_thresholds = [[55, 70,-30, -10,-35, -20]]

# ============ 初始化 ============
cam_block = camera.Camera(640, 480)
disp = display.Display()

device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

def uart_receive_thread(serial):
    global stuts, car_grab_ok, line_track_mode
    while True:
        data = serial.read()
        if data:
            try:
                decoded = data.decode("utf-8", errors="ignore").strip()
                if decoded:
                    stuts = decoded
                    
                    if "track_start" in decoded.lower():
                        line_track_mode = True
                        print("[UART RX] 收到track_start，进入自动循迹模式")
                    elif "track_stop" in decoded.lower():
                        line_track_mode = False
                        print("[UART RX] 收到track_stop，退出自动循迹模式")
                    
                    if "ok" in decoded.lower():
                        car_grab_ok = True
                        print("[UART RX] ✅ 收到有效的抓取完成ok！")
            except:
                pass
        time.sleep(0.01)

uart_thread = threading.Thread(target=uart_receive_thread, args=(serial,))
uart_thread.daemon = True
uart_thread.start()

def send_offset(dx, dy):
    global last_send_time, send_success, send_success_time, send_data
    
    if not system_active or waiting_ok:
        return
    
    current_time = time.time()
    if current_time - last_send_time < send_interval:
        return
    last_send_time = current_time
    
    dx = int(dx)
    dy = int(dy)
    data = f"{dx} {dy}\n"
    
    try:
        serial.write_str(data.encode('utf-8'))
        send_success = True
        send_success_time = current_time
        send_data = data
        print(f"[UART TX] X{dx:+d} Y{dy:+d}")
    except Exception as e:
        print(f"[UART TX] 发送失败: {e}")

def parse_qr_task(qr_text):
    global task_queue, task_parsed
    
    print(f"\\n[QR] {qr_text}")
    
    text = qr_text.lower().strip()
    color_names = {
        "red": "red", "红": "red", "红色": "red",
        "blue": "blue", "蓝": "blue", "蓝色": "blue",
        "yellow": "yellow", "黄": "yellow", "黄色": "yellow",
        "purple": "purple", "紫": "purple", "紫色": "purple",
        "pink": "pink", "粉": "pink", "粉色": "pink"
    }
    
    task_queue = []
    
    parts = text.split()
    if len(parts) >= 4:
        colors_part = []
        numbers_part = []
        for part in parts:
            if part in color_names:
                colors_part.append(color_names[part])
            elif part.isdigit():
                numbers_part.append(int(part))
        if colors_part and numbers_part and len(colors_part) == len(numbers_part):
            task_queue = list(zip(colors_part, numbers_part))
    
    if not task_queue:
        pattern = r'(红|蓝|黄|紫|粉)(?:色)?(\\d+)块?'
        matches = re.findall(pattern, qr_text)
        if matches:
            for color, count in matches:
                color_en = color_names.get(color)
                if color_en:
                    task_queue.append((color_en, int(count)))
    
    if not task_queue:
        pattern = r'(red|blue|yellow|purple|pink)(\\d+)'
        matches = re.findall(pattern, text)
        if matches:
            for color, count in matches:
                task_queue.append((color, int(count)))
    
    if task_queue:
        task_parsed = True
        cn_names = {"red": "红色", "blue": "蓝色", "yellow": "黄色", "purple": "紫色", "pink": "粉色"}
        print("[TASK] 解析成功:")
        for color_en, count in task_queue:
            print(f"  {cn_names[color_en]}: {count}块")
        return True
    
    print("[TASK] 解析失败")
    return False

def detect_green_line(img):
    """
    检测绿色线，返回 (dx, angle) 或 None
    dx: 线中心相对于图像中心的水平偏移（正=右，负=左）
    angle: 线的角度（正=向右弯，负=向左弯，0=直行）
    """
    blobs = img.find_blobs(green_line_thresholds, pixels_threshold=30, area_threshold=30, merge=True)
    
    if not blobs:
        return None
    
    mid_y = 240  # 图像中线（640x480）
    
    top_cx_sum = 0.0
    top_weight = 0
    bottom_cx_sum = 0.0
    bottom_weight = 0
    all_cx_sum = 0.0
    all_cy_sum = 0.0
    all_weight = 0
    
    for blob in blobs:
        cx, cy = blob[5], blob[6]
        weight = blob[4]  # 像素数
        
        all_cx_sum += cx * weight
        all_cy_sum += cy * weight
        all_weight += weight
        
        if cy < mid_y:
            top_cx_sum += cx * weight
            top_weight += weight
        else:
            bottom_cx_sum += cx * weight
            bottom_weight += weight
        
        img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_GREEN, 2)
    
    if all_weight == 0:
        return None
    
    overall_cx = all_cx_sum / all_weight
    overall_cy = all_cy_sum / all_weight
    
    dx = int(overall_cx - center_x)
    
    angle = 0
    if top_weight > 0 and bottom_weight > 0:
        top_avg = top_cx_sum / top_weight
        bottom_avg = bottom_cx_sum / bottom_weight
        angle = int(math.degrees(math.atan2(top_avg - bottom_avg, mid_y)))
    
    # 绘制线中心和方向
    img.draw_cross(int(overall_cx), int(overall_cy), image.COLOR_RED, size=15, thickness=2)
    if top_weight > 0 and bottom_weight > 0:
        top_avg = top_cx_sum / top_weight
        bottom_avg = bottom_cx_sum / bottom_weight
        img.draw_line(int(bottom_avg), mid_y, int(top_avg), 0, image.COLOR_YELLOW, 3)
    
    return dx, angle

def line_track_loop():
    global line_track_mode
    
    print("\n" + "="*50)
    print("自动循迹模式：绿色线跟踪")
    print("="*50)
    
    while line_track_mode and not app.need_exit():
        img = cam_block.read()
        
        result = detect_green_line(img)
        
        if result:
            dx, angle = result
            data = f"{dx} {angle}\n"
            try:
                serial.write_str(data)
            except:
                pass
            
            img.draw_string(10, 10, "自动循迹中", image.COLOR_GREEN, 1.5)
            img.draw_string(10, 30, f"dx={dx:+d} angle={angle:+d}", image.COLOR_GREEN, 1.5)
        else:
            data = "999 0\n"
            try:
                serial.write_str(data)
            except:
                pass
            
            img.draw_string(10, 10, "自动循迹中", image.COLOR_RED, 1.5)
            img.draw_string(10, 30, "绿线丢失!", image.COLOR_RED, 1.5)
        
        img.draw_cross(center_x, center_y, image.COLOR_WHITE, size=20, thickness=2)
        img.draw_circle(center_x, center_y, 8, image.COLOR_WHITE, 2)
        
        disp.show(img)
        time.sleep(0.02)

def phase_qr_scan():
    global task_parsed
    
    print("\\n" + "="*50)
    print("阶段1: 扫描二维码")
    print("="*50)
    
    while not task_parsed and not app.need_exit() and not line_track_mode:
        img = cam_block.read()
        qrcodes = img.find_qrcodes()
        
        for qr in qrcodes:
            corners = qr.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                             corners[(i+1)%4][0], corners[(i+1)%4][1],
                             image.COLOR_RED, 2)
            
            qr_text = qr.payload()
            img.draw_string(qr.x(), qr.y()-15, qr_text, image.COLOR_RED, 1.5)
            
            if parse_qr_task(qr_text):
                disp.show(img)
                time.sleep(1)
                return
        
        img.draw_string(10, 10, "扫描二维码中...", image.COLOR_GREEN, 1.5)
        disp.show(img)
        time.sleep(0.02)

def find_blocks_by_color(img, target_color):
    if target_color not in thresholds:
        return []
    
    all_blobs = img.find_blobs(thresholds[target_color], pixels_threshold=200, area_threshold=300, merge=True)
    if not all_blobs:
        return []
    
    valid_blocks = []
    for blob in all_blobs:
        x, y, w, h = blob[0], blob[1], blob[2], blob[3]
        cx, cy = blob[5], blob[6]
        area = blob.area()
        
        if area < 300 or w < 10 or h < 10 or w > 250 or h > 250:
            continue
        
        overlap = False
        for idx, existing in enumerate(valid_blocks):
            if abs(cx - existing['center_x']) < 50 and abs(cy - existing['center_y']) < 50:
                overlap = True
                if area > existing['area']:
                    valid_blocks[idx] = {
                        'x': x, 'y': y, 'width': w, 'height': h,
                        'center_x': cx, 'center_y': cy, 'area': area,
                        'distance': ((cx - center_x)**2 + (cy - center_y)**2)**0.5
                    }
                break
        
        if not overlap:
            valid_blocks.append({
                'x': x, 'y': y, 'width': w, 'height': h,
                'center_x': cx, 'center_y': cy, 'area': area,
                'distance': ((cx - center_x)**2 + (cy - center_y)**2)**0.5
            })
    
    return valid_blocks

def find_closest_block(img, target_color):
    blocks = find_blocks_by_color(img, target_color)
    if not blocks:
        return None
    # 过滤掉已抓过的物块（60像素半径内）
    if grabbed_positions:
        blocks = [b for b in blocks if not any(
            abs(b['center_x'] - gx) < 60 and abs(b['center_y'] - gy) < 60
            for gx, gy in grabbed_positions
        )]
    if not blocks:
        return None
    blocks.sort(key=lambda b: b['distance'])
    return blocks[0]

def find_locked_block(img, target_color, locked):
    blocks = find_blocks_by_color(img, target_color)
    if not blocks:
        return None
    
    best_match = None
    best_score = 999999
    
    for block in blocks:
        dx = block['center_x'] - locked['center_x']
        dy = block['center_y'] - locked['center_y']
        dw = block['width'] - locked['width']
        dh = block['height'] - locked['height']
        
        position_diff = (dx**2 + dy**2) ** 0.5
        size_diff = ((dw**2 + dh**2) ** 0.5) / max(locked['width'], locked['height'], 1)
        score = position_diff * 0.6 + size_diff * 100 * 0.4
        
        if score < best_score:
            best_score = score
            best_match = block
    
    if best_match and best_score < 80:
        return best_match
    return None

def draw_ui(img, block, color_en, grab_count, remaining_count, locked, align_cnt, lost_cnt):
    global send_success, send_success_time, send_data, stuts
    
    cn_names = {"red": "红色", "blue": "蓝色", "yellow": "黄色", "purple": "紫色", "pink": "粉色"}
    color_cn = cn_names.get(color_en, color_en)
    
    # 中心十字
    img.draw_cross(center_x, center_y, image.COLOR_WHITE, size=20, thickness=2)
    img.draw_circle(center_x, center_y, 8, image.COLOR_WHITE, 2)
    
    # 对准框
    box_color = image.COLOR_RED if waiting_ok else image.COLOR_GREEN
    img.draw_rect(center_x - ALIGN_THRESHOLD, center_y - ALIGN_THRESHOLD,
                  ALIGN_THRESHOLD * 2, ALIGN_THRESHOLD * 2, box_color, 2)
    
    # 状态栏
    img.draw_rect(0, 0, 640, 105, image.COLOR_BLACK, -1)
    img.draw_rect(0, 0, 640, 105, image.COLOR_WHITE, 1)
    
    img.draw_string(10, 5, f"目标: {color_cn} [{grab_count}/{remaining_count}]", image.COLOR_GREEN, 1.5)
    
    if waiting_ok:
        img.draw_string(400, 5, "⏳等待ok...", image.COLOR_YELLOW, 1.5)
    elif system_active:
        img.draw_string(500, 5, "●活动", image.COLOR_GREEN, 1.2)
    
    if locked:
        img.draw_string(550, 25, "🔒锁定", image.COLOR_RED, 1.0)
    
    if block:
        draw_color = color_draw.get(color_en, image.COLOR_GREEN)
        img.draw_rect(block['x'], block['y'], block['width'], block['height'], draw_color, 3)
        
        bx, by = int(block['center_x']), int(block['center_y'])
        img.draw_cross(bx, by, draw_color, size=10, thickness=2)
        img.draw_line(bx, by, center_x, center_y, image.COLOR_YELLOW, 1)
        
        dx = int(block['center_x'] - center_x)
        dy = int(block['center_y'] - center_y)
        
        offset_color = image.COLOR_GREEN if abs(dx) <= ALIGN_THRESHOLD and abs(dy) <= ALIGN_THRESHOLD else image.COLOR_YELLOW
        img.draw_string(10, 25, f"偏移: X{dx:+d} Y{dy:+d}", offset_color, 1.5)
        
        if waiting_ok:
            img.draw_string(10, 45, "✓ 已对准，等待小车ok...", image.COLOR_GREEN, 1.5)
        elif align_cnt > 0:
            img.draw_string(10, 45, f"对准中 [{align_cnt}/{ALIGN_COUNT_NEEDED}]", image.COLOR_GREEN, 1.5)
        else:
            img.draw_string(10, 45, "↔ 调整中...", image.COLOR_YELLOW, 1.5)
    else:
        img.draw_string(10, 25, "偏移: ---  ---", image.COLOR_RED, 1.5)
        img.draw_string(10, 45, "🔍 搜索中...", image.COLOR_RED, 1.5)
    
    current_time = time.time()
    if send_success and (current_time - send_success_time) < 1.0:
        img.draw_string(10, 65, f"📤 {send_data}", image.COLOR_GREEN, 1.5)
    if stuts:
        img.draw_string(10, 85, f"📥 {stuts}", image.COLOR_BLUE, 1.5)

def phase_detect_and_send():
    global current_color, remaining_count, grab_count, system_active
    global locked_block, align_count, lock_lost_count, waiting_ok, car_grab_ok, ok_wait_start
    global grabbed_positions, grabbed_block_pos
    
    cn_names = {"red": "红色", "blue": "蓝色", "yellow": "黄色", "purple": "紫色", "pink": "粉色"}
    
    print("\\n" + "="*50)
    print("阶段2: 物块识别与偏移发送")
    print("="*50)
    
    system_active = True
    print(f"[INFO] 对准条件: |dx|≤{ALIGN_THRESHOLD} |dy|≤{ALIGN_THRESHOLD}, 连续{ALIGN_COUNT_NEEDED}帧")
    print("[INFO] 对准后停止发偏移，等待小车ok")
    
    print("[INFO] 等待移动到物块区...")
    time.sleep(2)
    
    for idx, (color_en, count) in enumerate(task_queue):
        current_color = color_en
        remaining_count = count
        grab_count = 0
        
        locked_block = None
        align_count = 0
        lock_lost_count = 0
        waiting_ok = False
        car_grab_ok = False
        ok_wait_start = 0.0
        grabbed_positions.clear()
        
        color_cn = cn_names.get(color_en, color_en)
        print(f"\\n{'='*30}")
        print(f"[TASK] {color_cn}: {count}块 ({idx+1}/{len(task_queue)})")
        print(f"{'='*30}")
        
        while grab_count < remaining_count and not app.need_exit() and not line_track_mode:
            img = cam_block.read()
            
            # ====== 收到ok，抓取完成 ======
            if car_grab_ok:
                print(f"[COMPLETE] {color_cn} 第{grab_count+1}块 抓取完成!")
                if grabbed_block_pos:
                    grabbed_positions.append(grabbed_block_pos)
                    grabbed_block_pos = None
                grab_count += 1
                
                locked_block = None
                align_count = 0
                lock_lost_count = 0
                waiting_ok = False
                car_grab_ok = False
                
                if grab_count >= remaining_count:
                    print(f"[DONE] {color_cn} 全部完成!")
                    time.sleep(0.5)
                    break
                
                print(f"[NEXT] 准备下一块 ({grab_count}/{remaining_count})")
                time.sleep(0.3)
                continue
            
            # ====== 等待ok中，不发偏移 ======
            if waiting_ok:
                # ok 回传失败时超时默认成功，自动进入下一块
                if time.time() - ok_wait_start > GRAB_TIMEOUT_SEC:
                    print("[ALIGN] ok超时，默认抓取成功，进入下一块")
                    car_grab_ok = True
                    waiting_ok = False
                
                if locked_block is not None:
                    tracked = find_locked_block(img, color_en, locked_block)
                    if tracked:
                        locked_block = tracked
                
                draw_ui(img, locked_block, color_en, grab_count, remaining_count,
                       locked_block is not None, align_count, lock_lost_count)
                disp.show(img)
                time.sleep(0.01)
                continue
            
            # ====== 正常识别调整 ======
            block = None
            
            if locked_block is not None:
                tracked = find_locked_block(img, color_en, locked_block)
                if tracked:
                    locked_block = tracked
                    block = tracked
                    lock_lost_count = 0
                else:
                    lock_lost_count += 1
                    if lock_lost_count < LOCK_LOST_TOLERANCE:
                        block = locked_block
                    else:
                        print(f"[LOST] 丢失{lock_lost_count}帧，重新搜索")
                        locked_block = None
                        lock_lost_count = 0
                        align_count = 0
                        block = find_closest_block(img, color_en)
            else:
                block = find_closest_block(img, color_en)
                if block:
                    locked_block = block
                    lock_lost_count = 0
                    align_count = 0
                    print(f"[LOCK] 锁定{color_cn}: ({block['center_x']:.0f}, {block['center_y']:.0f})")
            
            if block:
                dx = int(block['center_x'] - center_x)
                dy = int(block['center_y'] - center_y)
                
                if abs(dx) <= ALIGN_THRESHOLD and abs(dy) <= ALIGN_THRESHOLD:
                    align_count += 1
                    if align_count >= ALIGN_COUNT_NEEDED:
                        print(f"[ALIGN] {color_cn} 第{grab_count+1}块 已对准! dx={dx:+d} dy={dy:+d}")
                        print("[ALIGN] 停止发偏移，等待小车抓取...")
                        if block:
                            grabbed_block_pos = (block['center_x'], block['center_y'])
                        send_offset(0, 0)
                        waiting_ok = True
                        car_grab_ok = False
                        ok_wait_start = time.time()
                else:
                    if align_count > 0:
                        print(f"[DEBUG] 超出范围! dx={dx:+d} dy={dy:+d}")
                    align_count = 0
                    send_offset(dx, dy)
            else:
                locked_block = None
                align_count = 0
                lock_lost_count = 0
                if grab_count < remaining_count:
                    send_offset(999, 999)
            
            draw_ui(img, block, color_en, grab_count, remaining_count,
                   locked_block is not None, align_count, lock_lost_count)
            disp.show(img)
            time.sleep(0.01)
    
    print("\\n" + "="*50)
    print("[TASK_COMPLETE] 所有任务完成")
    print("="*50)

    # 通知ESP32所有任务完成，自动退出抓取模式
    serial.write_str("finish\n")
    print("[UART TX] 发送finish → ESP32退出抓取模式")

    system_active = False
    waiting_ok = False
    current_color = None
    
    for _ in range(20):
        if app.need_exit():
            break
        img = cam_block.read()
        img.draw_rect(0, 0, 640, 480, image.COLOR_BLACK, -1)
        img.draw_string(180, 200, "所有任务完成!", image.COLOR_GREEN, 2.5)
        img.draw_string(200, 250, "系统已停止", image.COLOR_RED, 2)
        disp.show(img)
        time.sleep(0.1)
    
    print("\\n[DONE] 程序执行完毕")

print("\\n" + "="*50)
print("视觉伺服物块抓取系统")
print("="*50)

system_active = False
waiting_ok = False
car_grab_ok = False

while not app.need_exit():
    # 检查自动循迹模式
    if line_track_mode:
        line_track_loop()
        continue
    
    task_parsed = False
    grabbed_positions.clear()
    grabbed_block_pos = None
    system_active = False
    waiting_ok = False
    car_grab_ok = False
    
    phase_qr_scan()
    
    if task_parsed and len(task_queue) > 0:
        phase_detect_and_send()
    
    print("\n[INFO] 返回初始状态，等待扫描新二维码...")
    for _ in range(30):
        if app.need_exit() or line_track_mode:
            break
        img = cam_block.read()
        img.draw_string(180, 200, "扫描新二维码...", image.COLOR_GREEN, 2.5)
        disp.show(img)
        time.sleep(0.1)

print("\\n[END] 程序结束")