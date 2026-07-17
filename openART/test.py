
from machine import Pin
import sensor, image, math, time, struct
from machine import UART
from pyb import LED

# 测试引脚，根据你的实际情况改，这里用 J6，设为下拉
test_pin = Pin('M5', mode=Pin.IN)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(True)

sensor.set_hmirror(True)
sensor.skip_frames(times=200)
sensor.set_vflip(True)
sensor.skip_frames(times=200)
sensor.set_framerate(60)
sensor.skip_frames(times=200)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)

black = (0, 5, -40, 92, -20, 12)
while True: # 直接死循环找，直到找到完美的直接 return 跳出
    img = sensor.snapshot()
    mask = img.copy()
    mask.binary([black], invert=True)
    lines=mask.find_lines((12,110,136,11),threshold=1700)
    if lines:
        l = lines[0]
        img.draw_cross(l.x1(),l.y1())
        img.draw_cross(l.x2(),l.y2())
    '''mask = img.copy()
    mask.binary([black], invert=True)
    rects = mask.find_rects(threshold=15000)
    for r in rects:
        if r.w() * r.h() > 4000:
            corners = r.corners()
            # --- 检查是否所有角点都在图像内部 ---
            is_valid_rect = True
            for pt in corners:
                if not (0 <= pt[0] < sensor.width() and 0 <= pt[1] < sensor.height()):
                    is_valid_rect = False
                    break
            if not is_valid_rect:
                continue
            for i in range(4):                                          ####
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i+1)%4][0], corners[(i+1)%4][1],
                              color=(255, 0, 0), thickness=2)'''


