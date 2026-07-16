import sensor, image, math, time, struct
from machine import UART
from pyb import LED

# 初始化传感器
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
#用QQVGA角度会有较大误差
sensor.set_framesize(sensor.QVGA) # 320*240
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.skip_frames(times=200)
sensor.set_framerate(60)
sensor.skip_frames(times=200)

def sort_corners(corners):
    """
    四象限判别法，排序成：左上, 右上, 右下, 左下
    """
    # 按照 y 坐标排序，区分出上半部分(2个点)和下半部分(2个点)
    corners = sorted(corners, key=lambda p: p[1])
    top_points = corners[:2]
    bottom_points = corners[2:]
    # 在上半部分中，x 较小的是左上，x 较大的是右上
    top_points = sorted(top_points, key=lambda p: p[0])
    tl = top_points[0]
    tr = top_points[1]
    # 在下半部分中，x 较小的是左下，x 较大的是右下
    bottom_points = sorted(bottom_points, key=lambda p: p[0])
    bl = bottom_points[0]
    br = bottom_points[1]
    return [tl, tr, br, bl]
def init_grid_from_corners(corners, img_w, img_h, rows=12, cols=16):
    """
    根据给定的四个角点用透视变换计算网格点位置。
    """
    p0, p1, p2, p3 = sort_corners(corners)
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    #中间变量
    dx1 = x1 - x2
    dx2 = x3 - x2
    dx3 = x0 - x1 + x2 - x3
    dy1 = y1 - y2
    dy2 = y3 - y2
    dy3 = y0 - y1 + y2 - y3
    det = dx1 * dy2 - dx2 * dy1
    #如果 det == 0，说明没有透视形变，退化为仿射变换
    if det == 0:
        v1, v2 = 0.0, 0.0
    else:
        v1 = (dx3 * dy2 - dx2 * dy3) / det
        v2 = (dx1 * dy3 - dx3 * dy1) / det
    #求解映射系数(套公式)
    a1 = x1 - x0 + v1 * x1
    a2 = x3 - x0 + v2 * x3
    t1 = x0
    a3 = y1 - y0 + v1 * y1
    a4 = y3 - y0 + v2 * y3
    t2 = y0
    map_points = []
    # 遍历归一化网格 (0.0 ~ 1.0)
    for r in range(rows):
        for col in range(cols):
            # 取网格中心点归一化坐标
            u = (col + 0.5) / cols
            v = (r + 0.5) / rows
            # 引入了深度因子 w
            w = v1 * u + v2 * v + 1.0
            curr_x = int((a1 * u + a2 * v + t1) / w)
            curr_y = int((a3 * u + a4 * v + t2) / w)
            # 边界保护
            if 0 <= curr_x < img_w and 0 <= curr_y < img_h:
                map_points.append((curr_x, curr_y))
            else:
                map_points.append((-1,-1))
    return tuple(map_points)
def get_inv_perspective_coeffs(corners):
    """
    根据四个角点，计算出逆透视变换的 9 个核心系数。
    用于将图像像素坐标 (x,y) 还原为物理归一化坐标 (u,v)
    """
    p0, p1, p2, p3 = sort_corners(corners)
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    dx1 = x1 - x2
    dx2 = x3 - x2
    dx3 = x0 - x1 + x2 - x3
    dy1 = y1 - y2
    dy2 = y3 - y2
    dy3 = y0 - y1 + y2 - y3
    det = dx1 * dy2 - dx2 * dy1
    if det == 0:
        v1, v2 = 0.0, 0.0
    else:
        v1 = (dx3 * dy2 - dx2 * dy3) / det
        v2 = (dx1 * dy3 - dx3 * dy1) / det
    # 正向变换矩阵参数
    a1 = x1 - x0 + v1 * x1
    a2 = x3 - x0 + v2 * x3
    t1 = x0
    a3 = y1 - y0 + v1 * y1
    a4 = y3 - y0 + v2 * y3
    t2 = y0
    # 逆透视矩阵推导
    A = a4 - t2 * v2
    B = t1 * v2 - a2
    C = a2 * t2 - t1 * a4
    D = t2 * v1 - a3
    E = a1 - t1 * v1
    F = t1 * a3 - a1 * t2
    G = a3 * v2 - a4 * v1
    H = a2 * v1 - a1 * v2
    I = a1 * a4 - a2 * a3
    return (A, B, C, D, E, F, G, H, I)
def get_grid_colors(img, coord_map, L):
    """
    根据给定点列位置,在长L的正方形内取色。
    返回:(L,A,B) 元组列表。
    """
    results = [(0,0,0)]*LENS
    half_L = L // 2
    img_w = img.width()
    img_h = img.height()
    for i,pos in enumerate(coord_map):
        #roi = (pos[0]-half_L, pos[1]-half_L, L, L)
        rx = pos[0] - half_L
        ry = pos[1] - half_L
        rw = L
        rh = L
        # 处理左上角越界
        if rx < 0:
            rw += rx # 减去超出部分的宽度
            rx = 0
        if ry < 0:
            rh += ry # 减去超出部分的高度
            ry = 0
        # 处理右下角越界
        if rx + rw > img_w:
            rw = img_w - rx
        if ry + rh > img_h:
            rh = img_h - ry
        if rw > 0 and rh > 0:
            stats = img.get_statistics(roi=(rx, ry, rw, rh))
            results[i] = (stats.l_mean(), stats.a_mean(), stats.b_mean())
            #img.draw_rectangle(rx, ry, rw, rh,(255,0,255))           ####
    return results

def get_color_class(a, b):
    """
    根据欧氏距离判断所属类别
    0=空地 1=墙体 2=箱子 3=目的地 4=炸弹 5=小车 -1=未知
    """
    min_dist = 65530
    label = -1
    for name, center in COLOR_CENTERS.items():
        dist = (a - center[0])**2 + (b - center[1])**2
        if dist < min_dist:
            min_dist = dist
            label = name
    return label
def build_map_from_colors(colors):
    """
    输入: 元组序列 [(l,a,b), (l,a,b), ...]
    输出: 对应地图列表
    0=空地 1=墙体 2=箱子 3=目的地 4=炸弹 5=小车
    """
    maps = [0] * LENS
    for i,LAB in enumerate(colors):
        maps[i]=get_color_class(LAB[1], LAB[2])
    return maps

#经测试用roi得到的坐标就是原图像的坐标，无需转换
def get_and_update_car_loc(img, maps, L, inv_coeffs):
    """返回car在图像中的坐标和相对坐标,并在地图中更新小车近似位置"""
    flag = 0
    if maps.count(5) ==1:
        p = map_points[maps.index(5)]
        ROI = (p[0]-L, p[1]-L, 2*L, 2*L)
        blobs = img.find_blobs([car],roi=ROI, pixels_threshold=pixels_threshold)
    else:
        flag = 1
        blobs = img.find_blobs([green],roi=outer_rect, pixels_threshold=50)
        if not blobs:
            return -1
        ROI = (blobs[0].cx()-L, blobs[0].cy()-L, 2*L, 2*L)
        blobs = img.find_blobs([car],roi=ROI, pixels_threshold=pixels_threshold)
    if blobs:
        #print(len(blobs))                          ##
        x, y = blobs[0].cx(), blobs[0].cy()
        A, B, C, D, E, F, G, H, I = inv_coeffs
        w_inv = G * x + H * y + I
        if w_inv == 0: w_inv = 1e-6 # 防止极少数情况下的除零错误
        u = (A * x + B * y + C) / w_inv
        v = (D * x + E * y + F) / w_inv
        # 将 u, v 限制在 0.0 ~ 1.0 之间，防止小车出界导致数据错误
        u = max(0.0, min(1.0, u))
        v = max(0.0, min(1.0, v))
        if flag == 1:
            for i in range(len(maps)):
                if maps[i] == 5:
                    maps[i] = 0
            # 限制 col 和 row 最大值，防止数组越界 (比如 u=1.0 时 col=COLS)
            col = min(COLS - 1, int(u * COLS))
            row = min(ROWS - 1, int(v * ROWS))
            maps[col + COLS * row] = 5
        corners = blobs[0].min_corners()
        return (x, y) , corners , (u, v)    #自动打包为3元素元组
    else:
        return -1


def get_green_loc(img, pos, corners, L):
    """
    pos小车坐标 corners小车角点 输出方向向量的头尾端点 -1表示无绿色色块"""
    ROI = (pos[0]-L, pos[1]-L, 2*L, 2*L)
    blobs = img.find_blobs([green],roi=ROI, pixels_threshold=40)
    dists = {}
    if blobs:
        x,y = blobs[0].cx(), blobs[0].cy()
        #corners = blobs[0].min_corners()

        for c in range(4):
            dist = (x - corners[c][0])**2 + (y - corners[c][1])**2
            dists.update({c:dist})
        s = sorted(dists.items(),key=lambda x:x[1])
        #实测min_corners为顺时针输出！
        index_a = s[0][0]
        index_b = (index_a+1)%4
        Pa = corners[index_a]
        if index_b == s[1][0]:
            index_b = (index_a-1)%4
        Pb = corners[index_b]
        return (Pa,Pb)
    else:
        #G.on()                                             ##
        #time.sleep_ms(500)
        #G.off()
        return -1

def send_map_packet(maps):
    # 检查数据长度
    if len(maps) != LENS:
        return
    # 六进制压缩
    n = LENS//3
    compressed_bytes = bytearray(n)
    for i in range(n):
        # 每次取出3个数 高位*36 + 中位*6 + 低位
        val1 = maps[i*3]
        val2 = maps[i*3 + 1]
        val3 = maps[i*3 + 2]
        compressed_bytes[i] = val1 * 36 + val2 * 6 + val3
    # 组装数据包
    packet = bytearray([0xA5, n])
    packet.extend(compressed_bytes)
    checksum = (n + sum(compressed_bytes)) & 0xFF
    packet.append(checksum)
    uart.write(packet)

def send_2f_packet(car_loc):
    packet = bytearray([0xAA])
    float_bytes = struct.pack('<2f', *car_loc)
    packet.extend(float_bytes)
    # = sum(float_bytes) & 0xFF    #取低八位 0xFF=255
    #packet.append(checksum)
    uart.write(packet)

def send_float_packet(deg):
    packet = bytearray([0x5A])
    float_bytes = struct.pack('<f', deg)
    packet.extend(float_bytes)
    #checksum = sum(float_bytes) & 0xFF    #取低八位 0xFF=255
    #packet.append(checksum)
    print(packet)
    uart.write(packet)

def parse_uart_packet():
    """
    非阻塞解析串口数据，返回解析到的有效 flag。
    如果没有完整包或校验失败，返回 -1
    """
    global uart_buffer

    # 1. 把串口里现有的数据全部吸入缓冲区
    if uart.any():
        test = uart.read(uart.any())
        print(test)
        uart_buffer.extend(test)
    # 2. 只要缓冲区长度大于等于 4 (一个完整包的长度)，就尝试解析
    while len(uart_buffer) >= 4:
        # 寻找包头 0xA5 0x5A
        if uart_buffer[0] == 0xA5 and uart_buffer[1] == 0x5A:
            # 找到包头，计算校验和: Flag & 0xFF
            if uart_buffer[2] == uart_buffer[3]:
                valid_flag = uart_buffer[2]
                # 清空缓存
                uart_buffer = bytearray()
                return valid_flag
            else:
                # 包头对上了，但校验错乱 (说明数据被噪声破坏了)
                # 丢弃第一个字节，继续往后找
                uart_buffer = uart_buffer[1:]
        else:
            # 第一位不是包头，说明是垃圾数据，直接丢掉首字节
            uart_buffer = uart_buffer[1:]
    # 如果缓冲区不够 5 个字节，或者没找到有效包，返回 -1
    return -1

def draw_elem(maps, map_points):
    for i in range(len(maps)):
        if maps[i] == 0:
            img.draw_cross(map_points[i][0],map_points[i][1],(255,255,255),3,1)
        elif maps[i] == 1:
            img.draw_cross(map_points[i][0],map_points[i][1],(255,0,255),3,1)
        elif maps[i] == 2:
            img.draw_cross(map_points[i][0],map_points[i][1],(255,0,0),3,1)
        elif maps[i] == 3:
            img.draw_cross(map_points[i][0],map_points[i][1],(0,0,255),3,1)
        elif maps[i] == 4:
            img.draw_cross(map_points[i][0],map_points[i][1],(0,255,0),3,1)

def generate_mappoints():
    '''返回地图采样点+外界rect(x,y,w,h)用于roi'''
    while True: # 直接死循环找，直到找到完美的直接 return 跳出
        img = sensor.snapshot()
        mask = img.copy()
        mask.binary([black], invert=True)
        rects = mask.find_rects(threshold=20000)
        for r in rects:
            if r.w() * r.h() > 30000:
                map_corners = r.corners()
                # --- 检查是否所有角点都在图像内部 ---
                is_valid_rect = True
                for pt in map_corners:
                    if not (0 <= pt[0] < sensor.width() and 0 <= pt[1] < sensor.height()):
                        is_valid_rect = False
                        break
                if not is_valid_rect:
                    continue
                '''for i in range(4):                                          ####
                    img.draw_line(map_corners[i][0], map_corners[i][1],
                                  map_corners[(i+1)%4][0], map_corners[(i+1)%4][1],
                                  color=(255, 0, 0), thickness=2)'''

                map_points = init_grid_from_corners(map_corners, sensor.width(), sensor.height(), ROWS, COLS)
                if (-1, -1) in map_points:
                    continue
                inv_coeffs = get_inv_perspective_coeffs(map_corners)
                return map_points, r.rect(), inv_coeffs

#全局变量 调试用
#0=空地 1=墙体 2=箱子 3=目的地 4=炸弹 5=小车
ROWS = 12
COLS = 16
LENS = ROWS*COLS
pixels_threshold =  100    #QQVGA 30
black = (0, 10, -128, 127, -128, 127) #获取采样点用
count = 5  #角度求均值帧数

#wall = (17, 100, -16, 29, -42, 19)
# dark and normal
car = (0, 94, -90, -9, -53, 90)
green= (0, 94, -90, -40, -1, 90)
#cyan_thresholds = (47, 100, -128, -9, -114, 14)
#blue = (47, 94, -50, -19, -40, -8)
COLOR_CENTERS = {
    0: (67, -98),
    1: (5,-21),
    2: (-25, 76),
    3: (96, -64),
    4: (71, 55),
    5: (-45,13) #里中外三点取样
}
# light
#car = (0, 94, -90, -6, -17, 87)
#green= (0, 94, -90, -45, -5, 87)



'''log_file = open("/sd/fps_log.csv", "a+")
log_file.write("Time(ms),FPS\n")
last_log_time = time.ticks_ms() # 记录上次写入的时间点
record_count = 0 # 用于定期彻底保存文件'''
clock = time.clock()
diffx = [0]*count
diffy = [0]*count
uart = UART(12 , baudrate=115200)
uart_buffer = bytearray()
R = LED(1)    # 定义一个LED1   红灯
G = LED(2)  # 定义一个LED2   绿灯
B = LED(3)   # 定义一个LED3   蓝灯
Light = LED(4)  # 定义一个LED4   照明灯
last_maps = [0] * LENS
space_maps = [1] * LENS
last_spacemap = [1] * LENS

grid_spacing = 0
#grid_spacing = 18左右

wrong = 0   #地图错误

while grid_spacing < 10:
    map_points, outer_rect, inv_coeffs = generate_mappoints()
    grid_spacing = round(math.sqrt((map_points[15][0] - map_points[0][0])**2 + (map_points[15][1] - map_points[0][1])**2)/15)
    l = round(0.4*grid_spacing)     #采样矩形边长,故意大一点

while True:
    #flag = parse_uart_packet()
    flag = 0
    if uart.any():
        alls = uart.read(uart.any())
        flag = alls[-1]
        print(flag)
    img = sensor.snapshot()
    # 取色
    colors = get_grid_colors(img, map_points, l)
    # 识别
    maps = build_map_from_colors(colors)
    car_loc = get_and_update_car_loc(img, maps, grid_spacing, inv_coeffs)
    if maps[0] != 1:    #若未开始比赛,重新初始化
        space_maps = [1] * LENS
        last_spacemap = [1] * LENS
    else:
        for i in range(len(maps)):
            if space_maps[i] != 0:
                if maps[i] == 0:
                    space_maps[i] = 0
            else:
                if maps[i] == 1 or maps[i] == 3:
                    maps[i] = 0
    if last_maps != maps:
        last_maps = maps
        space_maps = last_spacemap
        continue
    last_spacemap = space_maps
    draw_elem(maps, map_points)                             ##

    if car_loc == -1:
        R.on()
        time.sleep_ms(200)
        R.off()
        grid_spacing = 0
        while grid_spacing < 10:
            map_points, outer_rect, inv_coeffs = generate_mappoints()
            grid_spacing = round(math.sqrt((map_points[15][0] - map_points[0][0])**2 + (map_points[15][1] - map_points[0][1])**2)/15)
            l = round(0.4*grid_spacing)
        #重新初始化
        space_maps = [1] * LENS
        last_spacemap = [1] * LENS
        wrong = 1
        continue
    # flag=0xFE 表示小车静止不动等待校正角度
    if flag == 0xFE:
        ab = get_green_loc(img, car_loc[0], car_loc[1], grid_spacing)
        if ab != -1:
            diffx[0]=ab[1][0]-ab[0][0]
            diffy[0]=ab[0][1]-ab[1][1]
            img.draw_line(ab[0][0],ab[0][1],ab[1][0],ab[1][1])
        for i in range(count-1):
            img = sensor.snapshot()
            ab = get_green_loc(img, car_loc[0], car_loc[1], grid_spacing)
            if ab != -1:
                diffx[i+1]=ab[1][0]-ab[0][0]
                diffy[i+1]=ab[0][1]-ab[1][1]
                img.draw_line(ab[0][0],ab[0][1],ab[1][0],ab[1][1],(255,0,255))
        deg = math.degrees(math.atan2(sum(diffy)-max(diffy)-min(diffy),
                sum(diffx)-max(diffx)-min(diffx)))
        print(deg)
        send_float_packet(deg)
    #发送map, car_loc, 能掉6帧！！！
    elif flag == 0xBB or wrong == 1:
        if maps.count(2) == maps.count(3):
            send_map_packet(maps)
            wrong = 0
        else:
            wrong = 1

    send_2f_packet(car_loc[2])
    print(car_loc[2][0], car_loc[2][1])




    """# clock.fps() 会根据两次 tick() 之间的时间差自动计算 FPS
    fps = clock.fps()
    current_time = time.ticks_ms()

    # --- 修复后的写入逻辑 ---
    # 检查时间差是否超过 100ms
    if time.ticks_diff(current_time, last_log_time) >= 100:
        log_file.write("%d,%.2f\n" % (current_time, fps))
        # flush 确保数据写入硬件，但不要太频繁
        log_file.flush()
        last_log_time = current_time
        record_count += 1

    # 每记录 100 条数据，重新开关一次文件（防止掉电导致文件损坏）
    if record_count >= 100:
        log_file.close()
        log_file = open("/sd/fps_log.csv", "a+")
        record_count = 0"""
