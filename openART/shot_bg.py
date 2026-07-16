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

# 初始化 GPIO 引脚
trigger_pin = Pin('M5', Pin.IN)

last_pin_state = 1  # 用于记录上一次的引脚状态
save_count = 1


while True:
    time.sleep_ms(150)
    img = sensor.snapshot()
    center_roi = (20, 0, 120, 120)
    canvas = img.copy(roi=center_roi)
    #img.draw_image(canvas, 0, 0)

    # ==========================================
    # GPIO 触发拍照逻辑
    current_pin_state = trigger_pin.value()
    if last_pin_state == 1 and current_pin_state == 0:
        # 拼接文件名并保存
        filename = "/sd/background/{}.jpg".format(save_count)
        canvas.save(filename)
        print(f"Saved: {save_count}") # 打印个数
        save_count += 1  # 序号加1
    last_pin_state = current_pin_state

