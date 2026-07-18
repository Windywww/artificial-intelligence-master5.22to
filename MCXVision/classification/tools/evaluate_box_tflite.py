import argparse
import json
from pathlib import Path

import numpy as np
import tensorflow as tf


def make_validation_dataset(data_dir: Path, image_size: int, batch_size: int):
    return tf.keras.utils.image_dataset_from_directory(
        str(data_dir),
        validation_split=0.2,
        subset="validation",
        seed=123,
        shuffle=True,
        image_size=(image_size, image_size),
        color_mode="rgb",
        batch_size=batch_size,
        label_mode="int",
    )


def evaluate(model_path: Path, dataset, batch_size: int):
    interpreter = tf.lite.Interpreter(model_path=str(model_path), num_threads=4)
    input_info = interpreter.get_input_details()[0]
    output_info = interpreter.get_output_details()[0]
    input_shape = list(input_info["shape"])
    input_shape[0] = batch_size
    interpreter.resize_tensor_input(input_info["index"], input_shape)
    interpreter.allocate_tensors()
    input_info = interpreter.get_input_details()[0]
    output_info = interpreter.get_output_details()[0]

    input_scale, input_zero = input_info["quantization"]
    if input_scale <= 0:
        raise RuntimeError(f"model is not quantized: {model_path}")

    correct = 0
    total = 0
    confusion = np.zeros((10, 10), dtype=np.int64)
    for images, labels in dataset:
        images = images.numpy()
        labels = labels.numpy().astype(np.int64)
        count = len(labels)
        quantized = np.rint(images / input_scale + input_zero)
        quantized = np.clip(quantized, -128, 127).astype(np.int8)
        if count < batch_size:
            padding = np.zeros((batch_size - count,) + quantized.shape[1:], dtype=np.int8)
            quantized = np.concatenate([quantized, padding], axis=0)

        interpreter.set_tensor(input_info["index"], quantized)
        interpreter.invoke()
        output = interpreter.get_tensor(output_info["index"])[:count]
        predictions = np.argmax(output, axis=1)
        correct += int(np.sum(predictions == labels))
        total += count
        for expected, predicted in zip(labels, predictions):
            confusion[expected, predicted] += 1

    per_class = []
    for index in range(10):
        class_total = int(np.sum(confusion[index]))
        per_class.append(
            float(confusion[index, index] / class_total) if class_total else 0.0
        )
    return {
        "model": str(model_path),
        "accuracy": correct / total,
        "correct": correct,
        "total": total,
        "per_class_recall": per_class,
        "confusion_matrix": confusion.tolist(),
        "input_quantization": [float(input_scale), int(input_zero)],
        "output_quantization": [
            float(output_info["quantization"][0]),
            int(output_info["quantization"][1]),
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("models", nargs="+")
    parser.add_argument(
        "--data-dir", default=r"D:\college\718\dataset\box\total"
    )
    parser.add_argument("--image-size", type=int, default=120)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--output")
    args = parser.parse_args()

    dataset = make_validation_dataset(
        Path(args.data_dir), args.image_size, args.batch_size
    ).cache()
    results = [
        evaluate(Path(model), dataset, args.batch_size) for model in args.models
    ]
    text = json.dumps(results, indent=2)
    print(text)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
