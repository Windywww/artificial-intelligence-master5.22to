import os
import random
import glob
import random
import cv2
from PIL import Image, ImageDraw  # 引入 ImageDraw 用于画遮挡块
import numpy as np
import albumentations as A

# ================= 配置区域 =================
# 文件夹路径
BG_DIR = "D:/college/718/dataset/background"  # 背景图片文件夹路径
BASE_FG_DIR = r"D:\college\718\dataset\box\shot_front" # 前景图片根路径
#BASE_FG_DIR = "D:\\college\\718\\智能视觉调试环境搭建软件\\上位机\\SmartCar_VR_V1.5\\image_class"
BASE_OUT_DIR = "D:/college/718/dataset/box/total"  # 合成数据集保存根路径

# 尺寸设置
MAX_FG_SIZE = 115
MIN_FG_SIZE = 50
# 背景尺寸设定
BG_WIDTH = 120
BG_HEIGHT = 120
CLASSES = [
    ("0", "00"),
    ("1", "01"),
    ("2", "02"),
    ("3", "03"),
    ("4", "04"),
    ("5", "05"),
    ("6", "06"),
    ("7", "07"),
    ("8", "08"),
    ("9", "09")
]
# ============================================

def cutout_fg_pil(fg_img, p=0.5, scale=(0.05, 0.2)):
    """
    专门针对 PIL RGBA 图像的 Cutout
    p: 触发概率
    scale: 遮挡面积占前景面积的比例范围
    """
    if random.random() > p:
        return fg_img
    w, h = fg_img.size
    area = w * h

    # 随机生成遮挡块的面积和长宽比
    target_area = random.uniform(*scale) * area
    aspect_ratio = random.uniform(0.1, 10.0)

    cut_h = int(np.sqrt(target_area * aspect_ratio))
    cut_w = int(np.sqrt(target_area / aspect_ratio))
    # 边界检查
    cut_h = min(cut_h, h - 5)
    cut_w = min(cut_w, w - 5)
    # 随机位置
    x = random.randint(0, w - cut_w)
    y = random.randint(0, h - cut_h)

    # 在 Alpha 通道上画一个透明矩形 (R, G, B, Alpha=0)
    draw = ImageDraw.Draw(fg_img)
    draw.rectangle([x, y, x + cut_w, y + cut_h], fill=(0, 0, 0, 0))
    return fg_img

fg_pipeline = A.Compose([
    A.RandomBrightnessContrast(brightness_limit=(-0.1, 0.1), contrast_limit=(-0.1,0.1), p=0.2),
    # 针对白平衡漂移的增强模块
    A.OneOf([
        # 1. 偏冷调 (模拟摄像头遇到暖屏，强行加蓝)
        # 压低红绿通道，抬高蓝通道
        A.RGBShift(r_shift_limit=(-2, 0), g_shift_limit=(-2, 0), b_shift_limit=(0, 5), p=1.0),
        # 2. 偏暖调 (模拟摄像头遇到冷屏，强行加黄/红)
        # 抬高红绿通道，压低蓝通道
        A.RGBShift(r_shift_limit=(0, 2), g_shift_limit=(0, 2), b_shift_limit=(-5, 0), p=1.0),
        # 3. 偏绿调 (非常经典的单片机 OV 摄像头偏色)
        # 抬高绿通道
        A.RGBShift(r_shift_limit=(-2, 2), g_shift_limit=(0, 5), b_shift_limit=(-2, 2), p=1.0),
        # 4. 全局随机轻微色相抖动 (作为补充)
        A.HueSaturationValue(hue_shift_limit=2, sat_shift_limit=2, val_shift_limit=0, p=1.0)
    ], p=0.1),
])
def apply_color_augmentation(fg_pil, pipeline, p=1.0):
    """
    对 PIL RGBA 图像应用 albumentations 颜色增强（不影响 Alpha 通道）
    fg_pil: PIL Image, mode='RGBA'
    pipeline: albumentations.Compose
    p: 应用增强的概率（全局开关）
    """
    if random.random() > p:
        return fg_pil
    # 转为 numpy 数组 (H, W, 4)
    fg_np = np.array(fg_pil)
    rgb = fg_np[:, :, :3]
    alpha = fg_np[:, :, 3]
    # 应用增强（albumentations 要求 uint8 格式）
    augmented = pipeline(image=rgb)
    rgb_aug = augmented['image']
    # 合并 alpha 通道
    fg_aug_np = np.dstack((rgb_aug, alpha))
    return Image.fromarray(fg_aug_np, mode='RGBA')

def get_image_paths(directory):
    extensions = '*.jpg'
    paths = []
    paths.extend(glob.glob(os.path.join(directory, extensions)))
    return paths

def generate_dataset():
    bg_paths = get_image_paths(BG_DIR)
    if not bg_paths:
        print("错误：背景文件夹中没有找到图片！")
        return
    for folder_name, class_id in CLASSES:
        fg_dir = os.path.join(BASE_FG_DIR, folder_name)
        out_dir = os.path.join(BASE_OUT_DIR, class_id)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        fg_paths = get_image_paths(fg_dir)
        if not fg_paths:
            print(f"警告：前景文件夹 {fg_dir} 中没有找到图片，跳过此类别！")
            continue

        total_fg = len(fg_paths)
        print(f"开始生成类别 [{class_id} - {folder_name}] 的数据集，原图 {total_fg} 张，预计生成 {total_fg} 张...")
        random.shuffle(fg_paths)
        # 记录当前生成的总图片数，用于命名
        generate_count = 0
        for fg_img_path in fg_paths:
            # 优化：在内层循环外读取前景原图，避免重复读取硬盘，极大提高生成速度
            fg_original = Image.open(fg_img_path)
            if fg_original.mode != 'RGBA':
                fg_original = fg_original.convert("RGBA")

            bg_img_path = random.choice(bg_paths)
            bg = Image.open(bg_img_path).convert("RGB")

            if bg.size != (BG_WIDTH, BG_HEIGHT):
                bg = bg.resize((BG_WIDTH, BG_HEIGHT), Image.Resampling.LANCZOS)

            # --- 1. 随机缩放前景 (保持长宽比) ---
            orig_w, orig_h = fg_original.size
            max_dim = max(orig_w, orig_h)  # 找出最长边

            # 随机确定最长边的目标尺寸
            target_longest_side = random.randint(MIN_FG_SIZE, MAX_FG_SIZE)

            # 计算缩放比例并得出新尺寸
            scale = target_longest_side / max_dim
            new_w = max(1, int(orig_w * scale))  # max(1, x) 防止极端情况变成0像素
            new_h = max(1, int(orig_h * scale))

            fg = fg_original.resize((new_w, new_h), Image.Resampling.LANCZOS)
            fg = apply_color_augmentation(fg, fg_pipeline)
            # 设定 50% 概率出现遮挡，遮挡面积为前景的 5%~20%
            fg = cutout_fg_pil(fg, p=0.5, scale=(0.05, 0.2))
            # 引入随机旋转
            #angle = random.uniform(-5.0, 5.0)
            #fg = fg.rotate(angle, expand=True, resample=Image.Resampling.BICUBIC)

            # 2. 将前景贴到背景上
            fg_w, fg_h = fg.size
            max_x = BG_WIDTH - fg_w
            max_y = BG_HEIGHT - fg_h
            pos_x = random.randint(0, max(0, max_x))
            pos_y = random.randint(0, max(0, max_y))
            bg.paste(fg, (pos_x, pos_y), mask=fg)

            # 3. 保存图片 (使用连续的 generate_count 命名)
            output_filename = f"real_{class_id}_{generate_count:04d}.jpg"
            bg.save(os.path.join(out_dir, output_filename), quality=95)
            generate_count += 1  # 计数器+1

        print(f"✅ 类别 [{class_id}] 完成！共合成 {generate_count} 张。")
    print("\n🎉 所有图片分类数据集生成完毕！")


if __name__ == "__main__":
    if not os.path.exists(BASE_OUT_DIR):
        os.makedirs(BASE_OUT_DIR)
    generate_dataset()