#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/kernels/neutron/neutron.h"

/*
 * 建立本项目允许使用的 TFLite Micro 算子表。
 * 只注册模型实际需要的算子可以减少 Flash 占用；模板参数 6 是最大算子数量。
 */
tflite::MicroOpResolver &CLASSIFICATION_GetOpsResolver()
{
    // static 保证 resolver 在函数返回后仍然存在，整个程序只初始化一份。
    static tflite::MicroMutableOpResolver<6> resolver;
    static bool initialized = false;
    if(!initialized)
    {
        // Slice/Reshape/Softmax 等由 CPU 执行，NEUTRON_GRAPH 是 NPU 网络主体。
        resolver.AddSlice();
        resolver.AddReshape();
        resolver.AddSoftmax();
        resolver.AddQuantize();
        resolver.AddDequantize();
        // 自定义算子名称必须与 FlatBuffer 中的 NEUTRON_GRAPH 节点匹配。
        resolver.AddCustom(tflite::GetString_NEUTRON_GRAPH(),
                           tflite::Register_NEUTRON_GRAPH());
        // 防止重复 Add 消耗 resolver 的固定容量。
        initialized = true;
    }
    return resolver;
}
