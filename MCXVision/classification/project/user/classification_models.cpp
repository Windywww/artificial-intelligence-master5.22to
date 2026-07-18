#include "classification_models.h"

#include <new>
#include <string.h>

#include "fsl_common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

extern "C"
{
extern const uint8_t num_model_data[];
extern const uint8_t box_model_data[];
}

extern tflite::MicroOpResolver &CLASSIFICATION_GetOpsResolver();

namespace mcxvision
{

namespace
{

static const size_t kTensorArenaSize = 190U * 1024U;
__ALIGNED(16) uint8_t s_tensor_arena[kTensorArenaSize];
__ALIGNED(16) uint8_t s_interpreter_storage[sizeof(tflite::MicroInterpreter)];
tflite::MicroInterpreter *s_interpreter = 0;
ModelKind s_active_model = static_cast<ModelKind>(-1);

int rounded_int(float value)
{
    return value >= 0.0f
        ? static_cast<int>(value + 0.5f)
        : static_cast<int>(value - 0.5f);
}

int clamp_int(int value, int low, int high)
{
    if(value < low) return low;
    if(value > high) return high;
    return value;
}

bool tensor_is_image(const TfLiteTensor *tensor)
{
    return tensor != 0 && tensor->dims != 0 && tensor->dims->size == 4
        && tensor->dims->data[0] == 1;
}

bool write_image_value(TfLiteTensor *tensor, size_t index, uint8_t pixel, ModelKind kind)
{
    if(tensor == 0 || index >= static_cast<size_t>(tensor->bytes))
    {
        return false;
    }

    const float real_value = kind == kDigitModel ? pixel / 255.0f : static_cast<float>(pixel);
    switch(tensor->type)
    {
        case kTfLiteInt8:
        {
            if(tensor->params.scale <= 0.0f)
            {
                return false;
            }
            const int quantized = rounded_int(real_value / tensor->params.scale)
                + tensor->params.zero_point;
            tensor->data.int8[index] = static_cast<int8_t>(clamp_int(quantized, -128, 127));
            return true;
        }
        case kTfLiteUInt8:
        {
            if(tensor->params.scale <= 0.0f)
            {
                tensor->data.uint8[index] = pixel;
                return true;
            }
            const int quantized = rounded_int(real_value / tensor->params.scale)
                + tensor->params.zero_point;
            tensor->data.uint8[index] = static_cast<uint8_t>(clamp_int(quantized, 0, 255));
            return true;
        }
        case kTfLiteFloat32:
            tensor->data.f[index] = real_value;
            return true;
        default:
            return false;
    }
}

float output_value(const TfLiteTensor *tensor, int index)
{
    switch(tensor->type)
    {
        case kTfLiteFloat32:
            return tensor->data.f[index];
        case kTfLiteInt8:
            return (tensor->data.int8[index] - tensor->params.zero_point) * tensor->params.scale;
        case kTfLiteUInt8:
            return (tensor->data.uint8[index] - tensor->params.zero_point) * tensor->params.scale;
        default:
            return -1.0f;
    }
}

} // namespace

bool model_select(ModelKind kind)
{
    if(s_interpreter != 0 && s_active_model == kind)
    {
        return true;
    }

    const uint8_t *model_data = kind == kDigitModel ? num_model_data : box_model_data;
    const tflite::Model *model = tflite::GetModel(model_data);
    if(model == 0 || model->version() != TFLITE_SCHEMA_VERSION)
    {
        return false;
    }

    if(s_interpreter != 0)
    {
        s_interpreter->~MicroInterpreter();
        s_interpreter = 0;
    }
    memset(s_tensor_arena, 0, sizeof(s_tensor_arena));

    s_interpreter = new (s_interpreter_storage) tflite::MicroInterpreter(
        model, CLASSIFICATION_GetOpsResolver(), s_tensor_arena, kTensorArenaSize);
    if(s_interpreter->AllocateTensors() != kTfLiteOk)
    {
        s_interpreter->~MicroInterpreter();
        s_interpreter = 0;
        return false;
    }

    s_active_model = kind;
    return true;
}

bool model_fill_digit(const uint8_t canvas[kDigitCanvasSize * kDigitCanvasSize])
{
    if(s_interpreter == 0 || s_active_model != kDigitModel)
    {
        return false;
    }
    TfLiteTensor *input = s_interpreter->input(0);
    if(!tensor_is_image(input)
        || input->dims->data[1] != kDigitCanvasSize
        || input->dims->data[2] != kDigitCanvasSize)
    {
        return false;
    }

    const int channels = input->dims->data[3];
    if(channels != 1 && channels != 3)
    {
        return false;
    }
    for(int i = 0; i < kDigitCanvasSize * kDigitCanvasSize; ++i)
    {
        for(int channel = 0; channel < channels; ++channel)
        {
            if(!write_image_value(input, static_cast<size_t>(i * channels + channel),
                                  canvas[i], kDigitModel))
            {
                return false;
            }
        }
    }
    return true;
}

bool model_fill_box(const uint16_t *frame, const Roi &roi)
{
    if(s_interpreter == 0 || s_active_model != kBoxModel)
    {
        return false;
    }
    TfLiteTensor *input = s_interpreter->input(0);
    if(!tensor_is_image(input))
    {
        return false;
    }

    const int height = input->dims->data[1];
    const int width = input->dims->data[2];
    const int channels = input->dims->data[3];
    if(width <= 0 || height <= 0 || (channels != 1 && channels != 3))
    {
        return false;
    }

    for(int y = 0; y < height; ++y)
    {
        const int source_y = roi.y + (y * roi.h) / height;
        for(int x = 0; x < width; ++x)
        {
            const int source_x = roi.x + (x * roi.w) / width;
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            read_oriented_rgb(frame, source_x, source_y, &red, &green, &blue);
            const size_t pixel_index = static_cast<size_t>(y * width + x) * channels;
            if(channels == 1)
            {
                const uint8_t gray = static_cast<uint8_t>((77U * red + 150U * green + 29U * blue) >> 8);
                if(!write_image_value(input, pixel_index, gray, kBoxModel)) return false;
            }
            else
            {
                if(!write_image_value(input, pixel_index + 0, red, kBoxModel)) return false;
                if(!write_image_value(input, pixel_index + 1, green, kBoxModel)) return false;
                if(!write_image_value(input, pixel_index + 2, blue, kBoxModel)) return false;
            }
        }
    }
    return true;
}

bool model_run(void)
{
    return s_interpreter != 0 && s_interpreter->Invoke() == kTfLiteOk;
}

bool model_top1(int *label, float *probability)
{
    if(s_interpreter == 0 || label == 0 || probability == 0)
    {
        return false;
    }
    TfLiteTensor *output = s_interpreter->output(0);
    if(output == 0 || output->dims == 0 || output->dims->size < 1)
    {
        return false;
    }

    int count = 1;
    for(int i = 0; i < output->dims->size; ++i)
    {
        count *= output->dims->data[i];
    }
    if(count <= 0)
    {
        return false;
    }

    int best_label = 0;
    float best_probability = output_value(output, 0);
    for(int i = 1; i < count; ++i)
    {
        const float value = output_value(output, i);
        if(value > best_probability)
        {
            best_probability = value;
            best_label = i;
        }
    }
    *label = best_label;
    *probability = best_probability;
    return true;
}

size_t model_arena_used(void)
{
    return s_interpreter == 0 ? 0U : s_interpreter->arena_used_bytes();
}

} // namespace mcxvision
