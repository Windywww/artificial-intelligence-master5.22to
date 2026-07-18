#ifndef MCXVISION_CLASSIFICATION_MODELS_H
#define MCXVISION_CLASSIFICATION_MODELS_H

#include <stddef.h>
#include <stdint.h>

#include "classification_core.h"

namespace mcxvision
{

enum ModelKind
{
    kDigitModel = 0,
    kBoxModel = 1
};

bool model_select(ModelKind kind);
bool model_fill_digit(const uint8_t canvas[kDigitCanvasSize * kDigitCanvasSize]);
bool model_fill_box(const uint16_t *frame, const Roi &roi);
bool model_run(void);
bool model_top1(int *label, float *probability);
size_t model_arena_used(void);

} // namespace mcxvision

#endif
