#ifndef MCXVISION_CLASSIFICATION_MODELS_H
#define MCXVISION_CLASSIFICATION_MODELS_H

/*
 * TFLite Micro/Neutron 推理封装的公共接口。
 * 调用顺序固定为 model_select() -> model_fill_*() -> model_run() -> model_top1()。
 * 两个模型共用同一块 Tensor Arena，切换模型时会重建解释器。
 */

#include <stddef.h>
#include <stdint.h>

#include "classification_core.h"

namespace mcxvision
{

// 用枚举表达当前选择的是数字模型还是 box 模型。
enum ModelKind
{
    kDigitModel = 0,
    kBoxModel = 1
};

// 选择并初始化模型；同模型连续调用会复用已有解释器。
bool model_select(ModelKind kind);
// 将 28x28 数字画布写入输入张量并按张量参数量化。
bool model_fill_digit(const uint8_t canvas[kDigitCanvasSize * kDigitCanvasSize]);
// 从 RGB565 帧缩放读取 ROI，并写入 box 模型输入张量。
bool model_fill_box(const uint16_t *frame, const Roi &roi);
// 执行一次推理；false 表示解释器未初始化或 Invoke 失败。
bool model_run(void);
// 找到输出张量最大值，返回类别下标和反量化后的概率。
bool model_top1(int *label, float *probability);
// 返回 TFLite Micro 实际使用的 Tensor Arena 字节数。
size_t model_arena_used(void);

} // namespace mcxvision

#endif
