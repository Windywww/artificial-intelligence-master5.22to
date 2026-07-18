#include "classification_core.h"

#include <string.h>

namespace mcxvision
{

const LabThreshold kBlackThreshold = {0, 31, -62, 43, -64, 44};
const LabThreshold kPurpleThreshold = {35, 88, 71, 127, -95, -45};
const Roi kCenterRoi = {20, 0, 120, 120};

namespace
{

uint8_t s_blob_mask[kImageWidth * kImageHeight];
uint16_t s_blob_queue[kImageWidth * kImageHeight];

float clamp_float(float value, float low, float high)
{
    if(value < low)
    {
        return low;
    }
    if(value > high)
    {
        return high;
    }
    return value;
}

float fast_cube_root(float value)
{
    if(value <= 0.0f)
    {
        return 0.0f;
    }

    union
    {
        float f;
        uint32_t i;
    } estimate;
    estimate.f = value;
    estimate.i = estimate.i / 3U + 709921077U;

    float result = estimate.f;
    result = (2.0f * result + value / (result * result)) / 3.0f;
    result = (2.0f * result + value / (result * result)) / 3.0f;
    return result;
}

float srgb_to_linear(float value)
{
    // Low-cost approximation of the IEC 61966-2-1 inverse transfer curve.
    return value * (value * (value * 0.305306011f + 0.682171111f) + 0.012522878f);
}

float lab_curve(float value)
{
    return value > 0.0088564517f
        ? fast_cube_root(value)
        : (7.787037f * value + 16.0f / 116.0f);
}

uint16_t byte_swap_16(uint16_t value)
{
    return static_cast<uint16_t>((value << 8) | (value >> 8));
}

bool inside_goal_mask(int x, int y)
{
    const bool lower = x >= 12 && x < 148 && y >= 110 && y < 115;
    const bool upper = x >= 19 && x < 137 && y >= 5 && y < 11;
    return lower || upper;
}

bool pixel_matches(const uint16_t *frame, int x, int y,
                   const LabThreshold &threshold, bool apply_goal_masks)
{
    if(apply_goal_masks && inside_goal_mask(x, y))
    {
        return false;
    }
    return matches_lab(frame, x, y, threshold);
}

int clamp_int(int value, int low, int high)
{
    if(value < low)
    {
        return low;
    }
    if(value > high)
    {
        return high;
    }
    return value;
}

} // namespace

UartPacketParser::UartPacketParser() : state_(0), flag_(0)
{
}

void UartPacketParser::reset()
{
    state_ = 0;
    flag_ = 0;
}

bool UartPacketParser::push(uint8_t byte, uint8_t *flag)
{
    switch(state_)
    {
        case 0:
            state_ = byte == 0xA5 ? 1 : 0;
            break;
        case 1:
            if(byte == 0x5A)
            {
                state_ = 2;
            }
            else
            {
                state_ = byte == 0xA5 ? 1 : 0;
            }
            break;
        case 2:
            flag_ = byte;
            state_ = 3;
            break;
        case 3:
            if(byte == flag_)
            {
                if(flag != 0)
                {
                    *flag = flag_;
                }
                reset();
                return true;
            }
            state_ = byte == 0xA5 ? 1 : 0;
            break;
        default:
            reset();
            break;
    }
    return false;
}

void rgb888_to_lab(uint8_t red, uint8_t green, uint8_t blue,
                   float *l, float *a, float *b)
{
    const float r = srgb_to_linear(red / 255.0f);
    const float g = srgb_to_linear(green / 255.0f);
    const float bl = srgb_to_linear(blue / 255.0f);

    const float x = (0.4124564f * r + 0.3575761f * g + 0.1804375f * bl) / 0.95047f;
    const float y = 0.2126729f * r + 0.7151522f * g + 0.0721750f * bl;
    const float z = (0.0193339f * r + 0.1191920f * g + 0.9503041f * bl) / 1.08883f;

    const float fx = lab_curve(x);
    const float fy = lab_curve(y);
    const float fz = lab_curve(z);

    if(l != 0)
    {
        *l = clamp_float(116.0f * fy - 16.0f, 0.0f, 100.0f);
    }
    if(a != 0)
    {
        *a = clamp_float(500.0f * (fx - fy), -128.0f, 127.0f);
    }
    if(b != 0)
    {
        *b = clamp_float(200.0f * (fy - fz), -128.0f, 127.0f);
    }
}

void read_oriented_rgb(const uint16_t *frame, int x, int y,
                       uint8_t *red, uint8_t *green, uint8_t *blue)
{
    const int source_x = kImageWidth - 1 - clamp_int(x, 0, kImageWidth - 1);
    const int source_y = kImageHeight - 1 - clamp_int(y, 0, kImageHeight - 1);
    const uint16_t rgb565 = byte_swap_16(frame[source_y * kImageWidth + source_x]);

    const uint8_t r5 = static_cast<uint8_t>((rgb565 >> 11) & 0x1F);
    const uint8_t g6 = static_cast<uint8_t>((rgb565 >> 5) & 0x3F);
    const uint8_t b5 = static_cast<uint8_t>(rgb565 & 0x1F);
    if(red != 0)
    {
        *red = static_cast<uint8_t>((r5 * 527U + 23U) >> 6);
    }
    if(green != 0)
    {
        *green = static_cast<uint8_t>((g6 * 259U + 33U) >> 6);
    }
    if(blue != 0)
    {
        *blue = static_cast<uint8_t>((b5 * 527U + 23U) >> 6);
    }
}

bool matches_lab(const uint16_t *frame, int x, int y,
                 const LabThreshold &threshold)
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    float l;
    float a;
    float b;
    read_oriented_rgb(frame, x, y, &red, &green, &blue);
    rgb888_to_lab(red, green, blue, &l, &a, &b);
    return l >= threshold.l_min && l <= threshold.l_max
        && a >= threshold.a_min && a <= threshold.a_max
        && b >= threshold.b_min && b <= threshold.b_max;
}

size_t find_blobs(const uint16_t *frame,
                  const LabThreshold &threshold,
                  const Roi &roi,
                  uint32_t min_area,
                  uint32_t max_area_exclusive,
                  bool apply_goal_masks,
                  Blob *output,
                  size_t output_capacity)
{
    memset(s_blob_mask, 0, sizeof(s_blob_mask));

    const int x0 = clamp_int(roi.x, 0, kImageWidth);
    const int y0 = clamp_int(roi.y, 0, kImageHeight);
    const int x1 = clamp_int(roi.x + roi.w, 0, kImageWidth);
    const int y1 = clamp_int(roi.y + roi.h, 0, kImageHeight);

    for(int y = y0; y < y1; ++y)
    {
        for(int x = x0; x < x1; ++x)
        {
            const int index = y * kImageWidth + x;
            s_blob_mask[index] = pixel_matches(frame, x, y, threshold, apply_goal_masks) ? 1U : 0U;
        }
    }

    size_t output_count = 0;
    for(int y = y0; y < y1; ++y)
    {
        for(int x = x0; x < x1; ++x)
        {
            const int start_index = y * kImageWidth + x;
            if(s_blob_mask[start_index] == 0)
            {
                continue;
            }

            uint32_t head = 0;
            uint32_t tail = 0;
            s_blob_queue[tail++] = static_cast<uint16_t>(start_index);
            s_blob_mask[start_index] = 0;

            uint32_t area = 0;
            int min_x = x;
            int max_x = x;
            int min_y = y;
            int max_y = y;

            while(head < tail)
            {
                const int index = s_blob_queue[head++];
                const int current_x = index % kImageWidth;
                const int current_y = index / kImageWidth;
                ++area;
                if(current_x < min_x) min_x = current_x;
                if(current_x > max_x) max_x = current_x;
                if(current_y < min_y) min_y = current_y;
                if(current_y > max_y) max_y = current_y;

                for(int dy = -1; dy <= 1; ++dy)
                {
                    const int next_y = current_y + dy;
                    if(next_y < y0 || next_y >= y1)
                    {
                        continue;
                    }
                    for(int dx = -1; dx <= 1; ++dx)
                    {
                        if(dx == 0 && dy == 0)
                        {
                            continue;
                        }
                        const int next_x = current_x + dx;
                        if(next_x < x0 || next_x >= x1)
                        {
                            continue;
                        }
                        const int next_index = next_y * kImageWidth + next_x;
                        if(s_blob_mask[next_index] != 0)
                        {
                            s_blob_mask[next_index] = 0;
                            s_blob_queue[tail++] = static_cast<uint16_t>(next_index);
                        }
                    }
                }
            }

            if(area >= min_area && area < max_area_exclusive)
            {
                if(output_count < output_capacity && output != 0)
                {
                    output[output_count].x = min_x;
                    output[output_count].y = min_y;
                    output[output_count].w = max_x - min_x + 1;
                    output[output_count].h = max_y - min_y + 1;
                    output[output_count].area = area;
                }
                ++output_count;
            }
        }
    }

    return output_count < output_capacity ? output_count : output_capacity;
}

void make_digit_canvas(const uint16_t *frame,
                       const Blob &blob,
                       uint8_t output[kDigitCanvasSize * kDigitCanvasSize])
{
    memset(output, 0, kDigitCanvasSize * kDigitCanvasSize);
    const int max_side = blob.w > blob.h ? blob.w : blob.h;
    if(max_side <= 0)
    {
        return;
    }

    const float scale = 20.0f / max_side;
    int scaled_w = static_cast<int>(blob.w * scale);
    int scaled_h = static_cast<int>(blob.h * scale);
    if(scaled_w < 1) scaled_w = 1;
    if(scaled_h < 1) scaled_h = 1;
    if(scaled_w > kDigitCanvasSize) scaled_w = kDigitCanvasSize;
    if(scaled_h > kDigitCanvasSize) scaled_h = kDigitCanvasSize;

    const int offset_x = static_cast<int>((kDigitCanvasSize - blob.w * scale) / 2.0f);
    const int offset_y = static_cast<int>((kDigitCanvasSize - blob.h * scale) / 2.0f);

    for(int dy = 0; dy < scaled_h; ++dy)
    {
        const int source_y = blob.y + (dy * blob.h) / scaled_h;
        const int output_y = offset_y + dy;
        if(output_y < 0 || output_y >= kDigitCanvasSize)
        {
            continue;
        }
        for(int dx = 0; dx < scaled_w; ++dx)
        {
            const int source_x = blob.x + (dx * blob.w) / scaled_w;
            const int output_x = offset_x + dx;
            if(output_x < 0 || output_x >= kDigitCanvasSize)
            {
                continue;
            }
            if(pixel_matches(frame, source_x, source_y, kBlackThreshold, true))
            {
                output[output_y * kDigitCanvasSize + output_x] = 255;
            }
        }
    }
}

} // namespace mcxvision
