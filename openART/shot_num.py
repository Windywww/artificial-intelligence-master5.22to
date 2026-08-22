import sensor, image, time, tf, gc, uos


BLACK_THRESHOLD = (0, 31, -62, 43, -64, 44)
NUM_ROI = (25, 8, 97, 109)
TARGET_DIGIT_SIZE = 20.0
CONFIDENCE_THRESHOLD = 0.55
SAVE_EVERY_VALID_PREDICTIONS = 15
SAVE_ROOT = '/sd/shot_num'
SAVE_EXTENSION = '.jpg'
class_save_counts = [0] * 10


def ensure_dir(path):
    try:
        uos.stat(path)
    except OSError:
        uos.mkdir(path)


def init_save_dirs():
    ensure_dir(SAVE_ROOT)
    for label in range(10):
        ensure_dir('%s/%d' % (SAVE_ROOT, label))
        class_save_counts[label] = get_initial_save_count(label)


def get_initial_save_count(label):
    class_dir = '%s/%d' % (SAVE_ROOT, label)
    next_index = 0
    try:
        filenames = uos.listdir(class_dir)
    except Exception as exc:
        print('list save dir error:', exc)
        return next_index

    for filename in filenames:
        if filename.startswith('num_') and filename.endswith(SAVE_EXTENSION):
            try:
                index = int(filename[4:10])
                if index >= next_index:
                    next_index = index + 1
            except Exception:
                pass
    return next_index


def get_next_filename(label):
    global class_save_counts
    class_dir = '%s/%d' % (SAVE_ROOT, label)
    while True:
        filename = '%s/num_%06d_%d%s' % (
            class_dir,
            class_save_counts[label],
            time.ticks_ms(),
            SAVE_EXTENSION,
        )
        try:
            uos.stat(filename)
            class_save_counts[label] += 1
        except OSError:
            class_save_counts[label] += 1
            return filename


def save_sample(canvas, label):
    try:
        filename = get_next_filename(label)
        canvas.save(filename)
        print('saved digit %d: %s' % (label, filename))
        return True
    except Exception as exc:
        print('digit image save error:', exc)
        return False


def draw_saved_prediction(img, blob, label, confidence, count):
    img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
    img.draw_string(
        30,
        10,
        'S%d L%d P%d' % (count, label, int(confidence * 100)),
        color=(255, 0, 0),
        scale=1,
    )


def build_model_input(img, blob):
    x, y, w, h = blob.rect()
    max_side = max(w, h)
    scale_factor = TARGET_DIGIT_SIZE / max_side

    canvas = img.copy(roi=(0, 0, 28, 28))
    canvas.draw_rectangle(
        0, 0, 28, 28,
        color=(255, 255, 255),
        fill=True,
    )

    offset_x = int((28 - w * scale_factor) / 2)
    offset_y = int((28 - h * scale_factor) / 2)
    canvas.draw_image(
        img,
        offset_x,
        offset_y,
        roi=(x, y, w, h),
        x_scale=scale_factor,
        y_scale=scale_factor,
    )
    canvas.binary([BLACK_THRESHOLD])
    canvas.to_grayscale()
    return canvas


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


num_path = '/sd/num_cls.tflite'
num_net = tf.load(
    num_path,
    load_to_fb=uos.stat(num_path)[6] > (gc.mem_free() - (64 * 1024)),
)
init_save_dirs()

valid_prediction_count = 0

while True:
    img = sensor.snapshot()

    # Keep the same border masking and ROI as classification.py.
    img.draw_rectangle((12, 113, 41, 7), fill=True)
    img.draw_rectangle((96, 113, 70, 7), fill=True)
    img.draw_rectangle((20, 0, 130, 13), fill=True)
    img.draw_rectangle((19, 10, 45, 6), fill=True)
    img.draw_rectangle((90, 10, 55, 6), fill=True)
    img.draw_rectangle((0, 0, 25, 120), fill=True)
    img.draw_rectangle((122, 0, 38, 120), fill=True)

    blobs = img.find_blobs(
        [BLACK_THRESHOLD],
        roi=NUM_ROI,
        area_threshold=2000,
    )
    if not blobs:
        continue

    img_cx = img.width() // 2
    img_cy = img.height() // 2
    best_blob = None
    best_dist = None
    for blob in blobs:
        dx = blob.cx() - img_cx
        dy = blob.cy() - img_cy
        distance = dx * dx + dy * dy
        if best_dist is None or distance < best_dist:
            best_dist = distance
            best_blob = blob

    if best_blob is None:
        continue

    model_input = build_model_input(img, best_blob)
    result = tf.classify(num_net, model_input)
    predictions = result[0].output()
    max_prob = max(predictions)
    label = predictions.index(max_prob)

    if max_prob < CONFIDENCE_THRESHOLD:
        print('invalid digit:', label, max_prob)
        continue

    valid_prediction_count += 1
    print('digit:', label, max_prob, valid_prediction_count)

    if valid_prediction_count % SAVE_EVERY_VALID_PREDICTIONS == 0:
        if save_sample(model_input, label):
            draw_saved_prediction(
                img,
                best_blob,
                label,
                max_prob,
                valid_prediction_count,
            )
