import sensor, image, time, tf, gc, uos
from machine import UART

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

'''0xA5=0b1010 0101 0x5A=0b0101 1010'''
def parse_uart_packet():
    """
    非阻塞解析串口数据，返回解析到的有效 flag。
    如果没有完整包或校验失败，返回 -1
    """
    global uart_buffer # 声明使用全局缓冲区，这是灵魂！
    # 1. 把串口新来的数据全部追加(追加而不是覆盖)到全局缓冲区中
    if uart.any():
        uart_buffer.extend(uart.read(uart.any()))
    # 2. 只要缓冲区长度大于等于 4 (一个完整包的长度)，就尝试解析
    while len(uart_buffer) >= 4:
        # 寻找包头 0xA5 0x5A
        if uart_buffer[0] == 0xA5 and uart_buffer[1] == 0x5A:
            # 找到包头，计算校验和: Flag & 0xFF
            if uart_buffer[2] == uart_buffer[3]:
                valid_flag = uart_buffer[2]
                uart_buffer = bytearray()
                return valid_flag
            else:
                # 包头对上了，但校验错乱 (说明数据被噪声破坏了)
                # 丢弃第一个字节，继续往后找
                uart_buffer = uart_buffer[1:]
        else:
            # 第一位不是包头，说明是垃圾数据，直接丢掉首字节
            uart_buffer = uart_buffer[1:]
    # 如果缓冲区不够 4 个字节，或者没找到有效包，返回 -1
    return -1
def send_int_packet(cls):
    uart.write(bytes([cls]))

black = (0, 31, -62, 43, -64, 44)
purple = (35, 88, 71, 127, -95, -45)
center_roi = (20, 0, 120, 120)
uart = UART(12 , baudrate=115200)
uart_buffer = bytearray()
TARGET_DIGIT_SIZE = 20.0
num_path = '/sd/num_cls.tflite'
num_net = tf.load(num_path, load_to_fb=uos.stat(num_path)[6] > (gc.mem_free() - (64*1024)))
box_path = '/sd/box_cls.tflite'
box_net = tf.load(box_path, load_to_fb=uos.stat(box_path)[6] > (gc.mem_free() - (64*1024)))

while(True):
    flag = parse_uart_packet()
    if flag != -1:
        print(flag)
    img = sensor.snapshot()
    #flag = 0xBB
    if flag == 0xFE:    #识别goal
        purples = img.find_blobs([purple],roi=center_roi, area_threshold=800)
        if purples:   #无分类
            print(0)
            send_int_packet(0)
        else:
            #找边框矩形面积在1500,9500之间的blob (QQVGA)
            img.draw_rectangle((12,110,136,5),fill=True)
            img.draw_rectangle((19,5,118,6),fill=True)
            blobs = img.find_blobs([black], area_threshold=1500, threshold_cb=lambda b: b.area() < 9500)
            if not blobs:
                send_int_packet(11)
                continue
            best_label = -1
            best_prob = 0.0
            for b in blobs:
                x, y, w, h = b.rect()
                max_side = max(w, h)
                scale_factor = TARGET_DIGIT_SIZE / max_side
                # 申请一个 28x28 的纯白画布
                canvas = img.copy(roi=(0, 0, 28, 28))
                canvas.draw_rectangle(0, 0, 28, 28, color=(255, 255, 255), fill=True)
                # 计算贴图偏移量
                offset_x = int((28 - w * scale_factor) / 2)
                offset_y = int((28 - h * scale_factor) / 2)
                # 从原图中抠取、缩放并贴入 28x28 画布
                #hint=image.BILINEAR
                canvas.draw_image(img, offset_x, offset_y, roi=(x, y, w, h),
                    x_scale=scale_factor, y_scale=scale_factor)
                # 二值化并转为灰度图喂给神经网络
                canvas.binary([black])
                canvas.to_grayscale()
                #img.draw_image(canvas, 0, 0, x_scale=2, y_scale=2)     ##
                # 绘制所有候选框(绿色)
                img.draw_rectangle(b.rect(), color=(0, 255, 0))         ##
                # 推理
                result = tf.classify(num_net, canvas)
                predictions = result[0].output()
                max_prob = max(predictions)
                if max_prob > 0.70:
                    current_label = predictions.index(max_prob)
                    if max_prob > best_prob:
                        best_prob = max_prob
                        best_label = current_label
                else:
                    continue
            if best_label != -1:
                send_int_packet(best_label+1)
                # 用红色显示最终的数字
                img.draw_string(40, 10, f"{best_label}={best_prob:.2f}", color=(255, 0, 0), scale=1)   ##
            else:
                send_int_packet(11)

    elif flag == 0xBB:  #识别box
        result = tf.classify(box_net, img, roi=center_roi)
        probs = result[0].output()
        max_prob = max(probs)
        label = probs.index(max_prob)
        img.draw_string(40, 10, f"{label} P:{max_prob:.2f}", color=(255, 0, 0), scale=2)    ##
        if max_prob < 0.50:
            send_int_packet(11)
            print('unknown')
        else:
            send_int_packet(label+1)
