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
black = (0, 5, -40, 92, -20, 12)

save_count = 81
n = 0
while True:
    img = sensor.snapshot()
    n+=1
    if n%60 == 0:
        filename = "/sd/{}.jpg".format(save_count)
        img.save(filename)
        print(f"Saved: {save_count}") # 打印个数
        save_count += 1


