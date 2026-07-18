#ifndef MCXVISION_CLASSIFICATION_CORE_H
#define MCXVISION_CLASSIFICATION_CORE_H

#include <stddef.h>
#include <stdint.h>

namespace mcxvision
{

static const int kImageWidth = 160;
static const int kImageHeight = 120;
static const int kDigitCanvasSize = 28;

struct Roi
{
    int x;
    int y;
    int w;
    int h;
};

struct Blob
{
    int x;
    int y;
    int w;
    int h;
    uint32_t area;
};

struct LabThreshold
{
    int l_min;
    int l_max;
    int a_min;
    int a_max;
    int b_min;
    int b_max;
};

extern const LabThreshold kBlackThreshold;
extern const LabThreshold kPurpleThreshold;
extern const Roi kCenterRoi;

class UartPacketParser
{
public:
    UartPacketParser();
    bool push(uint8_t byte, uint8_t *flag);
    void reset();

private:
    uint8_t state_;
    uint8_t flag_;
};

void rgb888_to_lab(uint8_t red, uint8_t green, uint8_t blue,
                   float *l, float *a, float *b);

// The OpenART script enables both horizontal mirror and vertical flip.
// Reading through this function applies the equivalent 180 degree rotation.
void read_oriented_rgb(const uint16_t *frame, int x, int y,
                       uint8_t *red, uint8_t *green, uint8_t *blue);

bool matches_lab(const uint16_t *frame, int x, int y,
                 const LabThreshold &threshold);

size_t find_blobs(const uint16_t *frame,
                  const LabThreshold &threshold,
                  const Roi &roi,
                  uint32_t min_area,
                  uint32_t max_area_exclusive,
                  bool apply_goal_masks,
                  Blob *output,
                  size_t output_capacity);

void make_digit_canvas(const uint16_t *frame,
                       const Blob &blob,
                       uint8_t output[kDigitCanvasSize * kDigitCanvasSize]);

} // namespace mcxvision

#endif
