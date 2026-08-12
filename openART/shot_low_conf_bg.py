import sensor, image, time, tf, gc, uos


CENTER_ROI = (20, 0, 120, 120)
CONFIDENCE_THRESHOLD = 0.60
SAVE_DIR = "/sd/background"
CAPTURE_INTERVAL_MS = 150


def ensure_save_dir():
    try:
        uos.stat(SAVE_DIR)
    except OSError:
        uos.mkdir(SAVE_DIR)


def get_next_save_count():
    save_count = 1
    while True:
        filename = "{}/{}.jpg".format(SAVE_DIR, save_count)
        try:
            uos.stat(filename)
            save_count += 1
        except OSError:
            return save_count


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(times=200)
sensor.set_auto_whitebal(False)
sensor.skip_frames(times=200)
sensor.set_auto_gain(False, gain_db=2.0)
sensor.set_auto_exposure(False, exposure_us=950)

sensor.set_hmirror(True)
sensor.skip_frames(times=200)
sensor.set_vflip(True)
sensor.skip_frames(times=200)
sensor.set_framerate(60)
sensor.skip_frames(times=200)

box_path = "/sd/box_cls.tflite"
box_net = tf.load(
    box_path,
    load_to_fb=uos.stat(box_path)[6] > (gc.mem_free() - (64 * 1024)),
)

ensure_save_dir()
save_count = get_next_save_count()

while True:
    time.sleep_ms(CAPTURE_INTERVAL_MS)
    img = sensor.snapshot()
    result = tf.classify(box_net, img, roi=CENTER_ROI)
    probs = result[0].output()
    max_prob = max(probs)
    label = probs.index(max_prob)

    if max_prob < CONFIDENCE_THRESHOLD:
        canvas = img.copy(roi=CENTER_ROI)
        filename = "{}/{}.jpg".format(SAVE_DIR, save_count)
        canvas.save(filename)
        print("Saved: {} P:{:.2f}".format(save_count, max_prob))
        save_count += 1

    img.draw_string(
        40,
        10,
        "{} P:{:.2f}".format(label, max_prob),
        color=(255, 0, 0),
        scale=2,
    )
