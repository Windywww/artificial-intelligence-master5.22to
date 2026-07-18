#include "../project/user/classification_core.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

namespace
{

uint16_t swap16(uint16_t value)
{
    return static_cast<uint16_t>((value << 8) | (value >> 8));
}

uint16_t camera_rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t rgb565 = static_cast<uint16_t>(((red >> 3) << 11)
        | ((green >> 2) << 5) | (blue >> 3));
    return swap16(rgb565);
}

void set_logical_pixel(uint16_t *frame, int x, int y,
                       uint8_t red, uint8_t green, uint8_t blue)
{
    const int source_x = mcxvision::kImageWidth - 1 - x;
    const int source_y = mcxvision::kImageHeight - 1 - y;
    frame[source_y * mcxvision::kImageWidth + source_x] = camera_rgb565(red, green, blue);
}

void fill_frame(uint16_t *frame, uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t value = camera_rgb565(red, green, blue);
    for(int i = 0; i < mcxvision::kImageWidth * mcxvision::kImageHeight; ++i)
    {
        frame[i] = value;
    }
}

void test_uart_parser()
{
    mcxvision::UartPacketParser parser;
    uint8_t flag = 0;
    assert(!parser.push(0x00, &flag));
    assert(!parser.push(0xA5, &flag));
    assert(!parser.push(0xA5, &flag));
    assert(!parser.push(0x5A, &flag));
    assert(!parser.push(0xFE, &flag));
    assert(parser.push(0xFE, &flag));
    assert(flag == 0xFE);

    assert(!parser.push(0xA5, &flag));
    assert(!parser.push(0x5A, &flag));
    assert(!parser.push(0xBB, &flag));
    assert(!parser.push(0xBA, &flag));
}

void test_color_and_blobs()
{
    static uint16_t frame[mcxvision::kImageWidth * mcxvision::kImageHeight];
    fill_frame(frame, 255, 255, 255);

    for(int y = 20; y < 50; ++y)
    {
        for(int x = 40; x < 70; ++x)
        {
            set_logical_pixel(frame, x, y, 190, 0, 190);
        }
    }

    mcxvision::Blob blob;
    size_t count = mcxvision::find_blobs(
        frame, mcxvision::kPurpleThreshold, mcxvision::kCenterRoi,
        800U, 0xFFFFFFFFU, false, &blob, 1U);
    assert(count == 1U);
    assert(blob.area == 900U);

    fill_frame(frame, 255, 255, 255);
    for(int y = 30; y < 70; ++y)
    {
        for(int x = 50; x < 90; ++x)
        {
            set_logical_pixel(frame, x, y, 0, 0, 0);
        }
    }
    const mcxvision::Roi full = {0, 0, mcxvision::kImageWidth, mcxvision::kImageHeight};
    count = mcxvision::find_blobs(frame, mcxvision::kBlackThreshold, full,
                                  1500U, 9500U, true, &blob, 1U);
    assert(count == 1U);
    assert(blob.x == 50 && blob.y == 30 && blob.w == 40 && blob.h == 40);

    uint8_t canvas[mcxvision::kDigitCanvasSize * mcxvision::kDigitCanvasSize];
    mcxvision::make_digit_canvas(frame, blob, canvas);
    int white_pixels = 0;
    for(size_t i = 0; i < sizeof(canvas); ++i)
    {
        white_pixels += canvas[i] == 255 ? 1 : 0;
    }
    assert(white_pixels == 400);
}

} // namespace

int main()
{
    test_uart_parser();
    test_color_and_blobs();
    puts("classification_core_test: PASS");
    return 0;
}
