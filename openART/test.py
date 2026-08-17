import sensor, image, math, time, struct
from machine import UART
from pyb import LED

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
black = (0, 5, -40, 92, -20, 12)
CENTER_ROI = (20, 0, 120, 120)
save_count = 0
n = 0
while True:
    img = sensor.snapshot()
    canvas = img.copy(roi=CENTER_ROI)
    n+=1
    if n%60 == 0:
        filename = "/sd/{}.jpg".format(save_count)
        canvas.save(filename)
        print(f"Saved: {save_count}") # 打印个数
        save_count += 1


