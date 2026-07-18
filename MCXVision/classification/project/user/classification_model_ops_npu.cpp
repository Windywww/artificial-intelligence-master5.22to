#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/kernels/neutron/neutron.h"

tflite::MicroOpResolver &CLASSIFICATION_GetOpsResolver()
{
    static tflite::MicroMutableOpResolver<6> resolver;
    static bool initialized = false;
    if(!initialized)
    {
        resolver.AddSlice();
        resolver.AddReshape();
        resolver.AddSoftmax();
        resolver.AddQuantize();
        resolver.AddDequantize();
        resolver.AddCustom(tflite::GetString_NEUTRON_GRAPH(),
                           tflite::Register_NEUTRON_GRAPH());
        initialized = true;
    }
    return resolver;
}
