import sensor, image, math, time, struct
from machine import UART
from pyb import LED

# 初始化传感器
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
#用QQVGA角度会有较大误差
sensor.set_framesize(sensor.QVGA) # 320*240
sensor.skip_frames(times=200)

sensor.set_auto_whitebal(False)
sensor.skip_frames(times=200)

# 比赛时直接跳过自动收敛，强制使用你在测试时觉得最完美的数值
sensor.set_auto_gain(False, gain_db=10.0)
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.skip_frames(time=500)
sensor.set_framerate(60)
sensor.skip_frames(times=200)


# ================= 畸变校正参数 =================
CAM_CX = 160.0  # 光心 X
CAM_CY = 120.0  # 光心 Y
CAM_FX = 200.0  # 焦距 X
CAM_FY = 200.0  # 焦距 Y
CAM_K1 = -0.02  # 径向畸变系数 k1 (桶形畸变通常为负)
CAM_K2 = 0.0    # 径向畸变系数 k2 (通常可忽略设为0)

def distort_point(ideal_x, ideal_y):
    """
    【正向畸变】将理想(无畸变)的物理网格坐标，扭曲回实际摄像头的畸变像素坐标。
    (用于：算出理想网格后，去畸变图里准确的位置取色)
    """
    x = (ideal_x - CAM_CX) / CAM_FX
    y = (ideal_y - CAM_CY) / CAM_FY
    r2 = x*x + y*y
    radial = 1.0 + CAM_K1 * r2 + CAM_K2 * r2 * r2
    x_dist = x * radial
    y_dist = y * radial
    return int(x_dist * CAM_FX + CAM_CX), int(y_dist * CAM_FY + CAM_CY)

def undistort_point(dist_x, dist_y):
    """
    【逆向去畸变】将摄像头拍到的畸变像素坐标，还原为理想(无畸变)的坐标。
    (用于：将识别到的畸变角点、小车坐标拉平，以计算准确的透视矩阵)
    注：代数求逆较难，这里使用3次迭代逼近，对MCU性能毫无压力。
    """
    x_d = (dist_x - CAM_CX) / CAM_FX
    y_d = (dist_y - CAM_CY) / CAM_FY
    x_guess, y_guess = x_d, y_d
    for _ in range(3): # 迭代3次基本收敛
        r2 = x_guess*x_guess + y_guess*y_guess
        radial = 1.0 + CAM_K1 * r2 + CAM_K2 * r2 * r2
        x_guess = x_d / radial
        y_guess = y_d / radial
    return (x_guess * CAM_FX + CAM_CX), (y_guess * CAM_FY + CAM_CY)
# =====================================================================

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
    p0, p1, p2, p3 = corners
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
            # 这里算出的是透视变换后的网格坐标
            w = v1 * u + v2 * v + 1.0
            ideal_x = (a1 * u + a2 * v + t1) / w
            ideal_y = (a3 * u + a4 * v + t2) / w

            # --- 关键修改：将理想坐标扭曲回实际畸变图像中的坐标 ---
            curr_x, curr_y = distort_point(ideal_x, ideal_y)

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
    0=空地 1=墙体 2=箱子 3=目的地 4=炸弹
    """
    min_dist = 65530
    label = -1
    for name, center in COLOR_CENTERS.items():
        dist = (a - center[0])**2 + (b - center[1])**2
        if dist < min_dist:
            min_dist = dist
            label = name
    if label == 6:
        label=0
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

# 经测试用 roi 得到的坐标就是原图像坐标，无需转换。
def clamp_roi(cx, cy, half_w, half_h, bounds):
    """生成限制在 bounds 内的 ROI。"""
    bx, by, bw, bh = bounds
    x0 = max(bx, int(cx - half_w))
    y0 = max(by, int(cy - half_h))
    x1 = min(bx + bw, int(cx + half_w))
    y1 = min(by + bh, int(cy + half_h))
    if x1 <= x0 or y1 <= y0:
        return bounds
    return (x0, y0, x1 - x0, y1 - y0)

def get_blob_center(blob):
    """优先使用亚像素质心；固件不支持时只检测一次并回退。"""
    global car_float_centroid_supported
    if CAR_USE_FLOAT_CENTROID and car_float_centroid_supported is not False:
        try:
            center = (blob.cxf(), blob.cyf())
            if car_float_centroid_supported is None:
                car_float_centroid_supported = True
                if CAR_ANGLE_DEBUG:
                    print("CAR centroid=cxf/cyf")
            return center
        except (AttributeError, TypeError):
            car_float_centroid_supported = False
            if CAR_ANGLE_DEBUG:
                print("CAR centroid=cx/cy fallback")
    return float(blob.cx()), float(blob.cy())

def find_best_car_pair(green_blobs, blue_blobs, L):
    """用小车的双色拼接结构选出最可信的蓝绿组合。"""
    best_pair = None
    best_score = 1000000
    for g in green_blobs:
        for b in blue_blobs:
            gx, gy = get_blob_center(g)
            bx, by = get_blob_center(b)
            dx = bx - gx
            dy = by - gy
            center_dist = math.sqrt(dx*dx + dy*dy)
            area_ratio = g.pixels() / max(1, b.pixels())

            x0 = min(g.x(), b.x())
            y0 = min(g.y(), b.y())
            x1 = max(g.x()+g.w(), b.x()+b.w())
            y1 = max(g.y()+g.h(), b.y()+b.h())
            union_w = x1 - x0
            union_h = y1 - y0

            if not (CAR_PAIR_AREA_RATIO_MIN < area_ratio < CAR_PAIR_AREA_RATIO_MAX):
                continue
            if not (CAR_PAIR_DIST_MIN*L < center_dist < CAR_PAIR_DIST_MAX*L):
                continue
            if union_w > CAR_PAIR_SIZE_MAX*L or union_h > CAR_PAIR_SIZE_MAX*L:
                continue

            # 面积越接近、中心间距越接近期望值、越靠近上一帧，得分越低。
            score = abs(area_ratio - 1.0) * L
            score += abs(center_dist - CAR_PAIR_DIST_EXPECTED*L)
            if last_car_pixel is not None:
                pair_x = (gx + bx) * 0.5
                pair_y = (gy + by) * 0.5
                score += CAR_TRACK_SCORE_WEIGHT * math.sqrt(
                    (pair_x-last_car_pixel[0])**2 + (pair_y-last_car_pixel[1])**2)
            if score < best_score:
                best_score = score
                best_pair = (g, b)
    return best_pair

def get_strict_pair_center(green_blob, blue_blob):
    """仅用严格蓝绿 blob 的几何信息计算小车中心。"""
    green_x, green_y = get_blob_center(green_blob)
    blue_x, blue_y = get_blob_center(blue_blob)
    pair_x = (green_x + blue_x) * 0.5
    pair_y = (green_y + blue_y) * 0.5

    # 联合边框中心不受两个色块面积比例影响，双质心中点对边缘缺损更不敏感。
    # 两者融合后不会再吸收 ROI 内的邻近墙体或箱子。
    x0 = min(green_blob.x(), blue_blob.x())
    y0 = min(green_blob.y(), blue_blob.y())
    x1 = max(green_blob.x()+green_blob.w(), blue_blob.x()+blue_blob.w())
    y1 = max(green_blob.y()+green_blob.h(), blue_blob.y()+blue_blob.h())
    bbox_x = (x0+x1)*0.5
    bbox_y = (y0+y1)*0.5

    center_x = STRICT_BBOX_CENTER_WEIGHT*bbox_x + (1.0-STRICT_BBOX_CENTER_WEIGHT)*pair_x
    center_y = STRICT_BBOX_CENTER_WEIGHT*bbox_y + (1.0-STRICT_BBOX_CENTER_WEIGHT)*pair_y
    return center_x, center_y

"""将小车图像坐标转换为虚拟地图归一化坐标。"""
def pixel_to_map_point(dist_x, dist_y, inv_coeffs):
    """将一个畸变图像点变换为俯视地图的归一化坐标。"""
    x, y = undistort_point(dist_x, dist_y)
    A, B, C, D, E, F, G, H, I = inv_coeffs
    w_inv = G*x + H*y + I
    if abs(w_inv) < 1e-6:
        return None
    u = (A*x + B*y + C) / w_inv
    v = (D*x + E*y + F) / w_inv
    return u, v

def pixel_to_angle_point(dist_x, dist_y, inv_coeffs):
    """将图像点变换为以方格边长为单位的俯视坐标。"""
    if CAR_ANGLE_USE_HOMOGRAPHY:
        map_point = pixel_to_map_point(dist_x, dist_y, inv_coeffs)
        if map_point is not None:
            return map_point[0]*COLS, map_point[1]*ROWS
    # 仅在矩阵异常或关闭单应性角度校正时回退到图像坐标。
    return dist_x, dist_y

def pixel_to_car_info(dist_x, dist_y, angle_deg, inv_coeffs):
    map_point = pixel_to_map_point(dist_x, dist_y, inv_coeffs)
    if map_point is None:
        return -1
    u, v = map_point
    if not (0.03 < u < 0.97 and 0.04 < v < 0.96):
        return -1
    return ((dist_x, dist_y), angle_deg, (u, v))

"""地图颜色分类不再负责寻找小车，只记录已跟踪到的小车格子。"""
def update_car_in_map(maps, car_info):
    for i in range(len(maps)):
        if maps[i] == 5:
            maps[i] = 0
    u, v = car_info[2]
    col = min(COLS-1, max(0, int(u*COLS)))
    row = min(ROWS-1, max(0, int(v*ROWS)))
    maps[col + COLS*row] = 5

def filter_car_angle(angle_rad):
    """在单位圆上进行 EMA，避免角度跨越 +/-180 度时产生跳变。"""
    global last_car_angle_sin, last_car_angle_cos
    raw_sin = math.sin(angle_rad)
    raw_cos = math.cos(angle_rad)
    if last_car_angle_sin is None:
        last_car_angle_sin = raw_sin
        last_car_angle_cos = raw_cos
    else:
        alpha = CAR_ANGLE_FILTER_ALPHA
        last_car_angle_sin += alpha*(raw_sin-last_car_angle_sin)
        last_car_angle_cos += alpha*(raw_cos-last_car_angle_cos)

        # 归一化可避免长时间滤波后向量长度逐渐缩小。
        norm = math.sqrt(last_car_angle_sin*last_car_angle_sin +
                         last_car_angle_cos*last_car_angle_cos)
        if norm > 1e-6:
            last_car_angle_sin /= norm
            last_car_angle_cos /= norm
    return math.atan2(last_car_angle_sin, last_car_angle_cos)

def get_line_angle_after_homography(x1, y1, x2, y2, inv_coeffs):
    """将线段端点映射到俯视方格坐标后计算其数学角度。"""
    p1 = pixel_to_angle_point(x1, y1, inv_coeffs)
    p2 = pixel_to_angle_point(x2, y2, inv_coeffs)
    axis_dx = p2[0]-p1[0]
    axis_dy = p1[1]-p2[1]
    if axis_dx*axis_dx + axis_dy*axis_dy <= 1e-8:
        return None
    return math.atan2(axis_dy, axis_dx)

def get_blob_major_axis_angle(blob, inv_coeffs):
    """在俯视方格坐标中计算 blob 长轴角度。"""
    try:
        # rotation() 保留浮点角度，小倾角时不会受整数轴线端点量化影响。
        center_x, center_y = get_blob_center(blob)
        raw_angle = -blob.rotation()
        radius = max(2.0, 0.25*max(blob.w(), blob.h()))
        image_dx = math.cos(raw_angle)*radius
        image_dy = -math.sin(raw_angle)*radius
        angle = get_line_angle_after_homography(
            center_x-image_dx, center_y-image_dy,
            center_x+image_dx, center_y+image_dy, inv_coeffs)
        if angle is not None:
            return angle
    except (AttributeError, TypeError):
        pass

    # 固件不支持 rotation() 时，再回退到整数端点的长轴线。
    try:
        line = blob.major_axis_line()
        return get_line_angle_after_homography(
            line[0], line[1], line[2], line[3], inv_coeffs)
    except (AttributeError, TypeError, IndexError):
        return None

def get_line_length(line):
    """计算 OpenMV 轴线四元组的长度。"""
    dx = line[2]-line[0]
    dy = line[3]-line[1]
    return math.sqrt(dx*dx + dy*dy)

def get_blob_axis_confidence(blob):
    """使用旋转不变的伸长度，排除接近正方形或噪声 blob 的方向。"""
    try:
        confidence = blob.elongation()
    except (AttributeError, TypeError):
        print("elongation false")
        try:
            major_length = get_line_length(blob.major_axis_line())
            minor_length = get_line_length(blob.minor_axis_line())
            confidence = 1.0-minor_length/max(1.0, major_length)
        except (AttributeError, TypeError, IndexError):
            # 不能用轴对齐的 w/h 回退；色块转到 45 度时该比值会失真。
            print("major_axis_line false")
            confidence = 0.0
    return max(0.0, min(1.0, confidence))

def align_axis_perpendicular(axis_angle, reference_angle):
    """选择与绿到蓝方向同半平面的长轴法线，消除 180 度二义性。"""
    candidate = axis_angle + math.pi*0.5
    if math.cos(candidate-reference_angle) < 0:
        candidate += math.pi
    return math.atan2(math.sin(candidate), math.cos(candidate))

def angle_difference(angle_a, angle_b):
    """返回两个角度之间的最短绝对距离，单位为弧度。"""
    return abs(math.atan2(math.sin(angle_a-angle_b),
                          math.cos(angle_a-angle_b)))

def get_fused_car_angle(green_blob, blue_blob, inv_coeffs):
    """融合双色质心方向与两个可靠半色块的长轴法线方向。"""
    green_x, green_y = get_blob_center(green_blob)
    blue_x, blue_y = get_blob_center(blue_blob)
    green_point = pixel_to_angle_point(green_x, green_y, inv_coeffs)
    blue_point = pixel_to_angle_point(blue_x, blue_y, inv_coeffs)
    dx = blue_point[0] - green_point[0]
    dy = green_point[1] - blue_point[1]
    centroid_angle = math.atan2(dy, dx)

    shape_weight_sum = 0.0
    shape_vector_x = 0.0
    shape_vector_y = 0.0
    rejected_axes = 0

    prior_angle = None
    if last_car_angle_sin is not None and last_car_angle_cos is not None:
        prior_angle = math.atan2(last_car_angle_sin, last_car_angle_cos)

    for blob in (green_blob, blue_blob):
        confidence = get_blob_axis_confidence(blob)
        if confidence < CAR_BLOB_AXIS_MIN_CONFIDENCE:
            continue
        axis_angle = get_blob_major_axis_angle(blob, inv_coeffs)
        if axis_angle is None:
            continue
        shape_angle = align_axis_perpendicular(axis_angle, centroid_angle)
        # 畸形 blob 可能给出伸长度很高但方向完全错误的长轴，因此还要依据
        # 上一帧滤波航向对形状角做门控。质心角不参与门控，仍可跟随真实急转。
        if prior_angle is not None and angle_difference(
                shape_angle, prior_angle) > CAR_BLOB_AXIS_MAX_PRIOR_ERROR:
            rejected_axes += 1
            continue
        weight = CAR_BLOB_AXIS_WEIGHT*confidence
        shape_vector_x += weight*math.cos(shape_angle)
        shape_vector_y += weight*math.sin(shape_angle)
        shape_weight_sum += weight

    centroid_weight = CAR_CENTROID_ANGLE_WEIGHT
    shape_weight_scale = 1.0
    shape_angle = None
    if shape_weight_sum > 0:
        shape_angle = math.atan2(shape_vector_y, shape_vector_x)
        if prior_angle is not None and angle_difference(
                centroid_angle, shape_angle) > CAR_ANGLE_SOURCE_DISAGREE:
            centroid_error = angle_difference(centroid_angle, prior_angle)
            shape_error = angle_difference(shape_angle, prior_angle)
            if centroid_error > shape_error + CAR_ANGLE_SOURCE_ERROR_MARGIN:
                centroid_weight *= CAR_ANGLE_CONFLICT_WEIGHT_SCALE
                shape_weight_scale = CAR_ANGLE_CONFLICT_SHAPE_BOOST
            elif shape_error > centroid_error + CAR_ANGLE_SOURCE_ERROR_MARGIN:
                shape_weight_scale = CAR_ANGLE_CONFLICT_WEIGHT_SCALE

    vector_x = centroid_weight*math.cos(centroid_angle)
    vector_y = centroid_weight*math.sin(centroid_angle)
    if shape_weight_sum > 0:
        vector_x += shape_weight_scale*shape_vector_x
        vector_y += shape_weight_scale*shape_vector_y
    fused_angle = math.atan2(vector_y, vector_x)

    if CAR_ANGLE_DEBUG:
        if shape_weight_sum > 0:
            print("ANGLE centroid=%.1f shape=%.1f fused=%.1f sw=%.2f cw=%.2f rej=%d" % (
                math.degrees(centroid_angle), math.degrees(shape_angle),
                math.degrees(fused_angle), shape_weight_sum*shape_weight_scale,
                centroid_weight, rejected_axes))
        else:
            print("ANGLE centroid=%.1f shape=NA fused=%.1f sw=0 cw=%.2f rej=%d" % (
                math.degrees(centroid_angle), math.degrees(fused_angle),
                centroid_weight, rejected_axes))
    return fused_angle

def get_and_update_car_info(img, maps, L, inv_coeffs):
    """通过双色配对和联合外形稳定获取小车位置、方向。"""
    global last_car_pixel, last_car_info, car_lost_frames

    if last_car_pixel is None:
        roi = outer_rect
    else:
        roi = clamp_roi(last_car_pixel[0], last_car_pixel[1],
                        CAR_TRACK_ROI_SCALE*L, CAR_TRACK_ROI_SCALE*L,
                        outer_rect)
    green_blobs = img.find_blobs(
        [greend, greenl], roi=roi, merge=True,
        margin=CAR_STRICT_MERGE_MARGIN,
        pixels_threshold=CAR_STRICT_PIXELS_THRESHOLD,
        area_threshold=CAR_STRICT_AREA_THRESHOLD)
    blue_blobs = img.find_blobs(
        [blued, bluel], roi=roi, merge=True,
        margin=CAR_STRICT_MERGE_MARGIN,
        pixels_threshold=CAR_STRICT_PIXELS_THRESHOLD,
        area_threshold=CAR_STRICT_AREA_THRESHOLD)

    pair = find_best_car_pair(green_blobs, blue_blobs, L)
    if pair is None and last_car_pixel is not None:
        # 局部跟踪失败后在全场补搜一次，避免运动过快时连续丢失。
        green_blobs = img.find_blobs(
            [greend, greenl], roi=outer_rect, merge=True,
            margin=CAR_STRICT_MERGE_MARGIN,
            pixels_threshold=CAR_STRICT_PIXELS_THRESHOLD,
            area_threshold=CAR_STRICT_AREA_THRESHOLD)
        blue_blobs = img.find_blobs(
            [blued, bluel], roi=outer_rect, merge=True,
            margin=CAR_STRICT_MERGE_MARGIN,
            pixels_threshold=CAR_STRICT_PIXELS_THRESHOLD,
            area_threshold=CAR_STRICT_AREA_THRESHOLD)
        pair = find_best_car_pair(green_blobs, blue_blobs, L)

    if pair is None:
        car_lost_frames += 1
        if last_car_info is not None and car_lost_frames <= CAR_MAX_HOLD_FRAMES:
            update_car_in_map(maps, last_car_info)
            return last_car_info
        return -1

    green_blob, blue_blob = pair
    if CAR_DEBUG:
        green_x, green_y = get_blob_center(green_blob)
        blue_x, blue_y = get_blob_center(blue_blob)
        pair_dx = blue_x - green_x
        pair_dy = blue_y - green_y
        print("CAR g=%d b=%d gp=%d bp=%d dist=%.1f" % (
            len(green_blobs), len(blue_blobs),
            green_blob.pixels(), blue_blob.pixels(),
            math.sqrt(pair_dx*pair_dx + pair_dy*pair_dy)))
    raw_x, raw_y = get_strict_pair_center(green_blob, blue_blob)
    if last_car_pixel is None:
        dist_x, dist_y = raw_x, raw_y
    else:
        # EMA滤波
        dist_x = last_car_pixel[0] + CAR_CENTER_FILTER_ALPHA*(raw_x-last_car_pixel[0])
        dist_y = last_car_pixel[1] + CAR_CENTER_FILTER_ALPHA*(raw_y-last_car_pixel[1])

    angle_rad = filter_car_angle(
        get_fused_car_angle(green_blob, blue_blob, inv_coeffs))
    angle_deg = math.degrees(angle_rad)
    #if CAR_ANGLE_DEBUG:
    #    print("ANGLE filtered=%.1f" % angle_deg)
    car_info = pixel_to_car_info(dist_x, dist_y, angle_deg, inv_coeffs)
    if car_info == -1:
        return -1

    last_car_pixel = (dist_x, dist_y)
    last_car_info = car_info
    car_lost_frames = 0
    update_car_in_map(maps, car_info)

    img.draw_cross(int(dist_x), int(dist_y), size=3, color=(255, 0, 0))
    green_x, green_y = get_blob_center(green_blob)
    blue_x, blue_y = get_blob_center(blue_blob)
    img.draw_line(int(green_x), int(green_y), int(blue_x), int(blue_y), color=(0, 0, 0))
    return car_info

def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def append_crc16(packet):
    crc = crc16_ccitt(packet)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)

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
    append_crc16(packet)
    uart.write(packet)

def send_2f_packet(car_loc):
    packet = bytearray([0xAA])
    float_bytes = struct.pack('<2f', *car_loc)
    packet.extend(float_bytes)
    append_crc16(packet)
    uart.write(packet)

def send_float_packet(deg):
    packet = bytearray([0x5A])
    float_bytes = struct.pack('<f', deg)
    packet.extend(float_bytes)
    append_crc16(packet)
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
            img.draw_cross(map_points[i][0],map_points[i][1],(0,0,0),3,1)
        elif maps[i] == 3:
            img.draw_cross(map_points[i][0],map_points[i][1],(0,0,0),3,1)
        elif maps[i] == 4:
            img.draw_cross(map_points[i][0],map_points[i][1],(0,255,0),3,1)

#全局变量 调试用
#0=空地 1=墙体 2=箱子 3=目的地 4=炸弹 5=小车
ROWS = 12
COLS = 16
LENS = ROWS*COLS
pixels_threshold =  60    #QQVGA 30
black0 = (0, 8, -128, 127, -128, 127)   #由空地获取角点
black1 = (0, 3, -128, 127, -128, 127)   #由边界墙获取角点
count = 5  #角度求均值帧数


'''car = (0, 94, -90, -9, -53, 90)
blue = (30, 93, -90, -3, -53, 5)
blued = (34, 81, -61, -4, -50, -4)
bluel = (32, 93, -65, -22, -44, 5)
green =(30, 93, -90, -38, -2, 85)
greend = (33, 85, -69, -36, 30, 69)
greenl = (30, 93, -90, -44, 21, 87)'''

car = (0, 100, -90, -9, -53, 90)
# 屏幕中心亮、四角暗，因此 L 放宽；蓝绿和黄色主要依靠 a/b 分离。
blued = (15, 100, -61, -4, -50, -4)
bluel = (15, 100, -65, -22, -44, 5)
greend = (15, 100, -69, -36, 30, 69)
greenl = (15, 100, -90, -44, 21, 87)

# ================= 小车检测参数 =================
# 在屏幕中心、四角分别采集蓝绿 LAB。优先调整 a/b，L 建议保持较宽，
# 避免屏幕中心亮、四角暗导致同一颜色被截断。
# blued/bluel/greend/greenl 用于严格双色检测，最终中心不再使用宽松阈值。
CAR_DEBUG = False
# alpha 越小越稳但延迟越大。0.65最合适
CAR_CENTER_FILTER_ALPHA = 0.65
# 测试阈值和几何参数时临时改为 True，稳定后务必关闭，避免串口打印拖慢帧率。
CAR_STRICT_PIXELS_THRESHOLD = 25
CAR_STRICT_AREA_THRESHOLD = 30
CAR_STRICT_MERGE_MARGIN = 4
# 车速较高而经常全场补搜时增大 TRACK_ROI_SCALE；误跟踪时减小。
CAR_TRACK_ROI_SCALE = 1.5


# 严格双 blob 联合边框中心的权重，其余权重分配给双质心中点。
# TODO(USER): 默认 0.5；边缘缺损明显时减小，两色面积变化明显时增大。
STRICT_BBOX_CENTER_WEIGHT = 0.50
# TODO(USER): 根据日志中的两个严格 blob 调整。中心距离通常约为半个小车边长。
CAR_PAIR_AREA_RATIO_MIN = 0.35
CAR_PAIR_AREA_RATIO_MAX = 2.80
CAR_PAIR_DIST_MIN = 0.18
CAR_PAIR_DIST_MAX = 0.85
CAR_PAIR_DIST_EXPECTED = 0.48
CAR_PAIR_SIZE_MAX = 1.45

CAR_TRACK_SCORE_WEIGHT = 0.35
# 角度使用单位圆 EMA；中心距仅 5~8 像素时，角度比中心坐标更容易抖动。
# TODO(USER): 20 FPS 下建议在 0.35~0.60 间测试；转向跟随慢则增大，静止抖动大则减小。
CAR_ANGLE_FILTER_ALPHA = 0.38
# 质心方向始终可用；仅当半色块具有足够伸长度时才融合其长轴法线。
CAR_ANGLE_DEBUG = False
CAR_USE_FLOAT_CENTROID = True
# 只变换质心和长轴端点，在俯视方格坐标中计算角度，不对整幅图像做 warp。
CAR_ANGLE_USE_HOMOGRAPHY = True
CAR_CENTROID_ANGLE_WEIGHT = 1.0
CAR_BLOB_AXIS_WEIGHT = 0.35
CAR_BLOB_AXIS_MIN_CONFIDENCE = 0.15
CAR_BLOB_AXIS_MAX_PRIOR_ERROR = math.radians(25.0)
# 两路角度冲突时，降低相对上一帧偏差更大的一路权重。
CAR_ANGLE_SOURCE_DISAGREE = math.radians(7.0)
CAR_ANGLE_SOURCE_ERROR_MARGIN = math.radians(3.0)
CAR_ANGLE_CONFLICT_WEIGHT_SCALE = 0.15
CAR_ANGLE_CONFLICT_SHAPE_BOOST = 3.0
# TODO(USER): 将 CAR_ANGLE_DEBUG 设为 True，测试 0/45/90/135/180 度。
# 正常情况下质心为主；质心异常时会自动临时放大形状角权重。
# 若错误长轴仍进入融合，将 CAR_BLOB_AXIS_MIN_CONFIDENCE 提高至 0.25。
# TODO(USER): 允许短暂识别失败时沿用旧坐标；20 FPS 下 3 帧约 150 ms。
CAR_MAX_HOLD_FRAMES = 3
# ==================================================
COLOR_CENTERS = {
    #0: (67, -98),
    0: (75,-104),
    1: (5,-21),
    2: (-25, 76),
    3: (94, -68),
    4: (71, 55),
    6: (40,-83),    #==0
    #容易和box混，舍弃吧
    #5: (-45,13) #里中外三点取样
}

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
last_car_pixel = None
last_car_info = None
last_car_angle_sin = None
last_car_angle_cos = None
car_float_centroid_supported = None
car_lost_frames = 0
bomb_count = 0
goal_count = 0
first = True        #是否为初始化后第一份地图
delay = False       #出发车区后是否延迟
wrong = 0   #地图不错误
grid_spacing = 0

def generate_mappoints(empty):
    """识别地图外框并更新采样点、ROI、单应矩阵和平均格距。"""
    global map_points, corners, inv_coeffs, outer_rect
    global grid_spacing, l
    while grid_spacing < 10:
        img = sensor.snapshot()
        #img.draw_rectangle(0,236,320,4,(0,0,0),fill=True)
        #img.draw_rectangle(0,0,4,240,(0,0,0),fill=True)
        mask = img.copy()
        if empty:
            mask.binary([black0], invert=True)
        else:
            mask.binary([black1], invert=True)
        rects = mask.find_rects(threshold=20000)
        for r in rects:
            if r.w() * r.h() > 49000:
                distorted_corners = r.corners()
                distorted_corners = sort_corners(distorted_corners)

                # 保留原有右下角经验补偿；原来的两次赋值等价于 (+1, -1)。
                x2, y2 = distorted_corners[2]
                distorted_corners[2] = (x2 + 1, y2 - 1)

                # --- 将这四个畸变角点还原为理想角点 ---
                new_corners = [undistort_point(px, py) for px, py in distorted_corners]
                # 边界检查使用实际拍摄到的畸变角点。
                is_valid_rect = True
                for pt in distorted_corners:
                    if not (0 <= pt[0] < sensor.width() and 0 <= pt[1] < sensor.height()):
                        is_valid_rect = False
                        break
                if not is_valid_rect:
                    continue

                new_map_points = init_grid_from_corners(
                    new_corners, sensor.width(), sensor.height(), ROWS, COLS)
                if (-1, -1) in new_map_points:
                    continue

                # 四条边分别除以对应格数后取平均，得到全场平均像素格距。
                top = math.sqrt((distorted_corners[1][0]-distorted_corners[0][0])**2 +
                                (distorted_corners[1][1]-distorted_corners[0][1])**2) / COLS
                bottom = math.sqrt((distorted_corners[2][0]-distorted_corners[3][0])**2 +
                                   (distorted_corners[2][1]-distorted_corners[3][1])**2) / COLS
                right = math.sqrt((distorted_corners[2][0]-distorted_corners[1][0])**2 +
                                  (distorted_corners[2][1]-distorted_corners[1][1])**2) / ROWS
                left = math.sqrt((distorted_corners[3][0]-distorted_corners[0][0])**2 +
                                 (distorted_corners[3][1]-distorted_corners[0][1])**2) / ROWS
                new_grid_spacing = (top + bottom + right + left) * 0.25
                if new_grid_spacing < 10:
                    continue

                # 全部结果有效后一次性更新，避免失败帧留下半初始化状态。
                map_points = new_map_points
                corners = tuple(new_corners)
                inv_coeffs = get_inv_perspective_coeffs(corners)
                outer_rect = r.rect()
                grid_spacing = new_grid_spacing
                l = max(2, round(0.4*grid_spacing))
                print("MAP INIT empty=%d spacing=%.1f" % (
                    1 if empty else 0, grid_spacing))
                return

generate_mappoints(True)
while True:
    flag = 0xFE
    if uart.any():
        alls = uart.read(uart.any())
        flag = alls[-1]
        print(flag)
    img = sensor.snapshot()

    #img.draw_rectangle(0,236,320,4,(0,0,0),fill=True)
    #img.draw_rectangle(0,0,4,240,(0,0,0),fill=True)
    for c in corners:
        draw_x, draw_y = distort_point(c[0], c[1])
        img.draw_cross(draw_x, draw_y, (255,255,255), 0)       ##

    # 取色
    colors = get_grid_colors(img, map_points, l)
    # 识别
    maps = build_map_from_colors(colors)
    car_info = get_and_update_car_info(img, maps, grid_spacing, inv_coeffs)
    if car_info == -1:
        R.on()
        time.sleep_ms(200)
        R.off()
        grid_spacing = 0
        generate_mappoints(False)
        #重新初始化
        space_maps = [1] * LENS
        last_spacemap = [1] * LENS
        wrong = 1
        first = True
        delay = False
        bomb_count = 0
        goal_count = 0
        last_car_pixel = None
        last_car_info = None
        last_car_angle_sin = None
        last_car_angle_cos = None
        car_lost_frames = 0
        continue
    if not maps.count(1):    #若未开始比赛,重新初始化
        print("wait for start...")
        first = True
        delay = False
        bomb_count = 0
        goal_count = 0
        space_maps = [1] * LENS
        last_spacemap = [1] * LENS
        wrong = 1
        send_2f_packet(car_info[2])
        continue
    # 修正5产生1的错误：若小车位于两格中间附近一小段，其中一格置为空地
    u, v = car_info[2]
    x,y = int(u*COLS), int(v*ROWS)
    #print(u*16,v*12)                       ##
    dx = u*COLS - x
    dy = v*ROWS - y
    if dx>0.8 and maps[x+COLS*y+1]==1:
        maps[x+1+COLS*y]=0
    elif dx<0.2 and maps[x+COLS*y-1]==1:
        maps[x-1+COLS*y]=0
    elif dy<0.2 and maps[x+COLS*(y-1)]==1:
        maps[x+COLS*(y-1)]=0
    elif dy>0.8 and maps[x+COLS*(y+1)]==1:
        maps[x+COLS*(y+1)]=0

    tmp_bomb_count = maps.count(4)
    tmp_goal_count = maps.count(3)
    if first == True and tmp_bomb_count+tmp_goal_count > 0:   #初始化数据记忆，更新space_maps
        if delay == False:
            time.sleep_ms(100)
            print("等待地图刷新")
            delay = True
            continue
        first = False
        space_maps = maps[:]
        bomb_count = tmp_bomb_count
        goal_count = tmp_goal_count
    #防止：4变两个3
    if bomb_count != tmp_bomb_count and goal_count == tmp_goal_count:   #有炸弹爆炸且goal数量正确
        space_maps = maps[:]       #更新space_maps
        bomb_count = tmp_bomb_count
    for i in range(192):
        m,n=maps[i],space_maps[i]
        if (m==3) and (n==0 or n==5 or n==4):
            maps[i] = 0

    '''if last_maps != maps:
        last_maps = maps[:]
        space_maps = last_spacemap
        wrong = 1
        continue
    last_spacemap = space_maps[:]'''

    #draw_elem(maps, map_points)                             ##

    # flag=0xFE 表示小车静止不动等待校正角度
    if flag == 0xFE:
        print(car_info[1])
        send_float_packet(car_info[1])
    elif flag == 0xBB:
        if maps.count(2) == maps.count(3):
            send_map_packet(maps)
            print("send map")           ##
    send_2f_packet(car_info[2])
