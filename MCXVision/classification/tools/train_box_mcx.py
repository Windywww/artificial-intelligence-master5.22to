import argparse
import json
from pathlib import Path

import tensorflow as tf


def load_datasets(data_dir: Path, image_size: int, batch_size: int):
    common = dict(
        directory=str(data_dir),
        validation_split=0.2,
        seed=123,
        image_size=(image_size, image_size),
        color_mode="rgb",
        batch_size=batch_size,
        label_mode="int",
    )
    train = tf.keras.utils.image_dataset_from_directory(
        subset="training", shuffle=True, **common
    )
    validation = tf.keras.utils.image_dataset_from_directory(
        subset="validation", shuffle=True, **common
    )
    class_names = train.class_names
    train = train.cache().shuffle(4096, seed=123).prefetch(tf.data.AUTOTUNE)
    validation = validation.cache().prefetch(tf.data.AUTOTUNE)
    return train, validation, class_names


def build_model(image_size: int, class_count: int):
    inputs = tf.keras.Input((image_size, image_size, 3), name="image")
    x = inputs
    for channels in (16, 32, 64, 128):
        x = tf.keras.layers.Conv2D(
            channels, 3, strides=2, padding="same", use_bias=False
        )(x)
        x = tf.keras.layers.BatchNormalization()(x)
        x = tf.keras.layers.ReLU()(x)
    for _ in range(2):
        x = tf.keras.layers.Conv2D(128, 3, padding="same", use_bias=False)(x)
        x = tf.keras.layers.BatchNormalization()(x)
        x = tf.keras.layers.ReLU()(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    x = tf.keras.layers.Dropout(0.10)(x)
    outputs = tf.keras.layers.Dense(class_count)(x)
    return tf.keras.Model(inputs, outputs, name="box_mcx_small")


def export_int8(model, train_dataset, output_path: Path):
    inference_model = tf.keras.Sequential([model, tf.keras.layers.Softmax()])

    def representative_data():
        for images, _ in train_dataset.take(100):
            yield [images]

    converter = tf.lite.TFLiteConverter.from_keras_model(inference_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_data
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    output_path.write_bytes(converter.convert())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data-dir", default=r"D:\college\718\dataset\box\total"
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--image-size", type=int, default=120)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    tf.keras.utils.set_random_seed(123)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    train, validation, class_names = load_datasets(
        Path(args.data_dir), args.image_size, args.batch_size
    )
    if class_names != [f"{index:02d}" for index in range(10)]:
        raise RuntimeError(f"unexpected class order: {class_names}")

    best_path = output_dir / "box_cls_mcx.keras"
    if args.resume and best_path.exists():
        model = tf.keras.models.load_model(best_path)
    else:
        model = build_model(args.image_size, len(class_names))
        model.compile(
            optimizer=tf.keras.optimizers.Adam(1e-3),
            loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
            metrics=["accuracy"],
        )
    callbacks = [
        tf.keras.callbacks.ModelCheckpoint(
            best_path, monitor="val_accuracy", save_best_only=True
        ),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor="val_accuracy", mode="max", factor=0.35, patience=2,
            min_lr=1e-5, verbose=1
        ),
        tf.keras.callbacks.EarlyStopping(
            monitor="val_accuracy", mode="max", patience=6,
            restore_best_weights=True, verbose=1
        ),
    ]
    history = model.fit(
        train,
        validation_data=validation,
        epochs=args.epochs,
        callbacks=callbacks,
        verbose=2,
    )
    model = tf.keras.models.load_model(best_path)
    loss, accuracy = model.evaluate(validation, verbose=0)
    export_int8(model, train, output_dir / "box_cls_mcx.tflite")

    report = {
        "classes": class_names,
        "image_size": args.image_size,
        "channels": [16, 32, 64, 128],
        "downsampling": "stride-2 convolution",
        "tail_convolution_blocks": 2,
        "online_augmentation": False,
        "validation_loss": float(loss),
        "validation_accuracy": float(accuracy),
        "epochs_completed": len(history.history["loss"]),
        "best_validation_accuracy": float(max(history.history["val_accuracy"])),
    }
    (output_dir / "box_cls_mcx_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
