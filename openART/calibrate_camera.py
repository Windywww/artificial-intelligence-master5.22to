"""使用张正友棋盘法标定相机，并导出 OpenCV/OpenMV 参数。"""

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="OpenCV 棋盘格相机标定")
    parser.add_argument("images", help="图片目录或 glob，例如 photos 或 photos/*.bmp")
    parser.add_argument("--cols", type=int, default=9, help="横向内角点数量，默认 9")
    parser.add_argument("--rows", type=int, default=7, help="纵向内角点数量，默认 7")
    parser.add_argument(
        "--square-size",
        type=float,
        default=1.0,
        help="单格实际边长，单位自定；只影响平移向量尺度",
    )
    parser.add_argument(
        "--output", default="camera_calibration.json", help="标定结果 JSON 文件"
    )
    parser.add_argument(
        "--preview-dir",
        default=None,
        help="可选：保存成功检测到角点的预览图",
    )
    parser.add_argument(
        "--reject-error",
        type=float,
        default=1.0,
        help="剔除单图重投影误差超过此像素值的照片；设为 0 禁用",
    )
    return parser.parse_args()


def collect_images(source):
    path = Path(source)
    if path.is_dir():
        files = []
        for extension in ("*.bmp", "*.png", "*.jpg", "*.jpeg", "*.tif", "*.tiff"):
            files.extend(path.glob(extension))
        return sorted(str(file) for file in files)
    return sorted(glob.glob(source))


def make_object_points(cols, rows, square_size):
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= square_size
    return points


def detect_corners(files, pattern_size, object_template, preview_dir=None):
    object_points = []
    image_points = []
    accepted_files = []
    image_size = None

    if preview_dir:
        Path(preview_dir).mkdir(parents=True, exist_ok=True)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.001,
    )

    for filename in files:
        image = cv2.imread(filename)
        if image is None:
            print("跳过无法读取的图片:", filename)
            continue

        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            print("跳过分辨率不一致的图片:", filename, current_size)
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found:
            print("未找到完整棋盘角点:", filename)
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted_files.append(filename)

        if preview_dir:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, pattern_size, corners, True)
            output = Path(preview_dir) / Path(filename).name
            cv2.imwrite(str(output), preview)

    return object_points, image_points, accepted_files, image_size


def calibrate(object_points, image_points, image_size):
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    errors = []
    for object_set, image_set, rotation, translation in zip(
        object_points, image_points, rotations, translations
    ):
        projected, _ = cv2.projectPoints(
            object_set, rotation, translation, matrix, distortion
        )
        residual = image_set.reshape(-1, 2) - projected.reshape(-1, 2)
        error = np.mean(np.linalg.norm(residual, axis=1))
        errors.append(float(error))
    return rms, matrix, distortion, rotations, translations, errors


def reject_outliers(object_points, image_points, files, errors, threshold):
    if threshold <= 0:
        return object_points, image_points, files, []
    keep = [index for index, error in enumerate(errors) if error <= threshold]
    rejected = [files[index] for index, error in enumerate(errors) if error > threshold]
    if len(keep) < 5:
        print("高误差图片剔除后不足 5 张，本次不执行剔除。")
        return object_points, image_points, files, []
    return (
        [object_points[index] for index in keep],
        [image_points[index] for index in keep],
        [files[index] for index in keep],
        rejected,
    )


def distortion_values(distortion):
    values = distortion.ravel().tolist()
    values.extend([0.0] * (5 - len(values)))
    return values[:5]


def save_result(output, image_size, pattern_size, square_size, rms, matrix,
                distortion, files, errors, rejected):
    k1, k2, p1, p2, k3 = distortion_values(distortion)
    result = {
        "image_size": list(image_size),
        "pattern_inner_corners": list(pattern_size),
        "square_size": square_size,
        "rms": float(rms),
        "mean_reprojection_error": float(np.mean(errors)),
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.ravel().tolist(),
        "opencv_distortion": {"k1": k1, "k2": k2, "p1": p1, "p2": p2, "k3": k3},
        "images": [
            {"file": filename, "reprojection_error": error}
            for filename, error in zip(files, errors)
        ],
        "rejected_images": rejected,
    }
    Path(output).write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n标定完成")
    print("有效图片数:", len(files))
    print("RMS:", float(rms))
    print("平均重投影误差: %.4f px" % np.mean(errors))
    print("相机矩阵:\n", matrix)
    print("畸变系数:", distortion.ravel())
    print("\n可写入 map_detect.py 的参数:")
    print("CAM_FX = %.8f" % matrix[0, 0])
    print("CAM_FY = %.8f" % matrix[1, 1])
    print("CAM_CX = %.8f" % matrix[0, 2])
    print("CAM_CY = %.8f" % matrix[1, 2])
    print("CAM_K1 = %.10f" % k1)
    print("CAM_K2 = %.10f" % k2)
    print("# OpenCV 额外参数: P1=%.10f P2=%.10f K3=%.10f" % (p1, p2, k3))
    print("结果已保存到:", output)

    if abs(p1) > 0.001 or abs(p2) > 0.001 or abs(k3) > 0.01:
        print("警告: P1/P2/K3 不可忽略，当前 OpenMV 两径向系数模型可能不够准确。")


def main():
    args = parse_args()
    files = collect_images(args.images)
    if not files:
        raise SystemExit("没有找到标定图片。")

    pattern_size = (args.cols, args.rows)
    template = make_object_points(args.cols, args.rows, args.square_size)
    object_points, image_points, accepted, image_size = detect_corners(
        files, pattern_size, template, args.preview_dir
    )
    if len(accepted) < 5:
        raise SystemExit("成功检测棋盘角点的图片不足 5 张，建议至少准备 15～25 张。")

    first = calibrate(object_points, image_points, image_size)
    object_points, image_points, accepted, rejected = reject_outliers(
        object_points, image_points, accepted, first[-1], args.reject_error
    )
    final = calibrate(object_points, image_points, image_size)
    rms, matrix, distortion, _, _, errors = final
    save_result(
        args.output,
        image_size,
        pattern_size,
        args.square_size,
        rms,
        matrix,
        distortion,
        accepted,
        errors,
        rejected,
    )


if __name__ == "__main__":
    main()
