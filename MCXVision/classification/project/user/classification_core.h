#ifndef MCXVISION_CLASSIFICATION_CORE_H
#define MCXVISION_CLASSIFICATION_CORE_H

/*
 * 图像预处理与串口协议的公共接口。
 * 这个文件只声明外部可以调用的内容，具体实现位于 classification_core.cpp。
 * 所有声明放在 mcxvision 命名空间中，避免与 SDK 中的同名符号冲突。
 */

#include <stddef.h>
#include <stdint.h>

namespace mcxvision
{

// SCC8660 QQVGA 输出 160x120；数字模型输入固定为 28x28。
static const int kImageWidth = 160;
static const int kImageHeight = 120;
static const int kDigitCanvasSize = 28;

// 感兴趣区域（ROI）：x/y 是左上角，w/h 是宽和高，单位为像素。
struct Roi
{
    int x;
    int y;
    int w;
    int h;
};

// 颜色连通区域的外接矩形和实际像素面积。w*h 与 area 通常不相等。
struct Blob
{
    int x;
    int y;
    int w;
    int h;
    uint32_t area;
};

// CIELAB 颜色空间的闭区间阈值，每个分量都必须落在 min..max 内。
struct LabThreshold
{
    int l_min;
    int l_max;
    int a_min;
    int a_max;
    int b_min;
    int b_max;
};

// 阈值和中心 ROI 在 .cpp 中只保存一份，这里用 extern 提供给其他文件访问。
extern const LabThreshold kBlackThreshold;
extern const LabThreshold kPurpleThreshold;
extern const Roi kCenterRoi;

/*
 * 逐字节解析 A5 5A flag flag 串口包的有限状态机。
 * push() 每收到一个字节调用一次，只有完整包且两个 flag 相同时才返回 true。
 */
class UartPacketParser
{
public:
    UartPacketParser();
    bool push(uint8_t byte, uint8_t *flag);
    void reset();

private:
    uint8_t state_; // 已匹配到协议的第几个字节，取值 0..3。
    uint8_t flag_;  // 暂存第一个 flag，供第 4 字节重复校验。
};

// RGB888 转换为 D65 白点下的 CIELAB；输出指针可为 0，表示忽略该分量。
void rgb888_to_lab(uint8_t red, uint8_t green, uint8_t blue,
                   float *l, float *a, float *b);

/*
 * 从摄像头 RGB565 帧读取像素并转换成 RGB888。OpenART 同时开启水平镜像和垂直翻转，
 * 这里等价为旋转 180 度；越界坐标会被限制到图像边缘。
 */
void read_oriented_rgb(const uint16_t *frame, int x, int y,
                       uint8_t *red, uint8_t *green, uint8_t *blue);

// 判断像素是否落入给定 LAB 阈值。
bool matches_lab(const uint16_t *frame, int x, int y,
                 const LabThreshold &threshold);

/*
 * 在 roi 内寻找满足颜色阈值的 8 邻域连通区域。只输出
 * min_area <= area < max_area_exclusive 的区域，返回值不会超过 output_capacity。
 */
size_t find_blobs(const uint16_t *frame,
                  const LabThreshold &threshold,
                  const Roi &roi,
                  uint32_t min_area,
                  uint32_t max_area_exclusive,
                  bool apply_goal_masks,
                  Blob *output,
                  size_t output_capacity);

// 将黑色数字区域等比例缩放、居中到 28x28 灰度画布，背景为 0、数字像素为 255。
void make_digit_canvas(const uint16_t *frame,
                       const Blob &blob,
                       uint8_t output[kDigitCanvasSize * kDigitCanvasSize]);

} // namespace mcxvision

#endif
