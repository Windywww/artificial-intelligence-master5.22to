import sensor, image, time, tf, gc, uos
from machine import UART, Pin

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(times=200)
sensor.set_auto_whitebal(False)
sensor.skip_frames(times=200)
# 比赛时直接跳过自动收敛，强制使用你在测试时觉得最完美的数值
sensor.set_auto_gain(False, gain_db=2.0)
sensor.set_auto_exposure(False, exposure_us=950)

sensor.set_hmirror(True)
sensor.skip_frames(times=200)
sensor.set_vflip(True)
sensor.skip_frames(times=200)
sensor.set_framerate(60)
sensor.skip_frames(times=200)
clock = time.clock()
black = (0, 27, -128, 127, -128, 127)
white = (37, 100, -9, 18, -50, 23)
yellow = (53, 100, -32, -10, 59, 95)
purple = (35, 88, 71, 127, -95, -45)
center_roi = (20, 0, 120, 120)
# ==========================================
# 硬件配置区
# ==========================================
# 初始化 GPIO 引脚，假设使用 P3 引脚。
# 因为你的需求是“读取1则拍照”，所以配置为输入模式 (Pin.IN)。
# 加入下拉电阻 (Pin.PULL_DOWN) 可以保证按键没按下时，引脚稳定为 0，防止静电干扰误触发。
trigger_pin = Pin('M5', Pin.IN)
last_pin_state = 1  # 用于记录上一次的引脚状态
save_count = 1

while True:
    time.sleep_ms(150)
    img = sensor.snapshot()
    blobs = img.find_blobs([black], invert=True, area_threshold=3000, threshold_cb=lambda b: b.area() < 12000)
    if blobs:
        b = blobs[0]
        #img.draw_rectangle(b.rect())
        canvas = img.copy(roi=b.rect())

        # ==========================================
        # GPIO 触发拍照逻辑
        # ==========================================
        current_pin_state = trigger_pin.value()
        if last_pin_state == 1 and current_pin_state == 0:
            # 拼接文件名并保存
            filename = str(save_count) + ".jpg"
            canvas.save(filename)
            print("Saved: " + filename) # 在终端打印提示
            save_count += 1  # 序号加1
        last_pin_state = current_pin_state

    else:
        blobs = img.find_blobs([white], invert=True, area_threshold=4000, threshold_cb=lambda b: b.area() < 12000)
        if blobs:
            b = blobs[0]
            #img.draw_rectangle(b.rect())
            canvas = img.copy(roi=b.rect())

            # ==========================================
            # GPIO 触发拍照逻辑
            # ==========================================
            current_pin_state = trigger_pin.value()
            if last_pin_state == 1 and current_pin_state == 0:
                # 拼接文件名并保存
                filename = str(save_count) + ".jpg"
                canvas.save(filename)
                print("Saved: " + filename) # 在终端打印提示
                save_count += 1  # 序号加1
            last_pin_state = current_pin_state
        else:
            last_pin_state = trigger_pin.value()
