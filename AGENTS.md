# MCXVision 仓库导航与 Neutron 内存验证

## 目录结构

### `MCXVision/classification/`

这是 OpenART `classification.py` 迁移到逐飞 MCXVision 官方 C/C++ 工程的目录：

- `project/user/`：MCX 用户代码和模型资源。
  - `classification_core.cpp/.h`：串口协议、图像预处理、颜色连通域和分类流程。
  - `classification_models.cpp/.h`：TFLite Micro/Neutron 模型加载与推理封装。
  - `classification_model_ops_npu.cpp`：Neutron 自定义算子注册。
  - `classification_model_data.s`：通过 `.incbin` 将模型放入片内 Flash。
  - `main.cpp`：摄像头、串口和分类任务入口。
  - `models/`：部署用 `.tflite` 和对应报告。
- `project/mdk/`：可直接 Rebuild、Download 和 Debug 的 Keil MDK 工程。
- `libraries/`：该工程所需的板级代码、CMSIS、MCX SDK、eIQ/TFLite Micro、
  Neutron 和逐飞驱动；必需的预编译 `.a`/`.lib` 也在这里。
- `tests/`：不依赖 MCX SDK 的图像核心单元测试。
- `tools/`：训练、TFLite 评估和模型比较脚本。
- `training_output/`、`training_output_v2/`、`training_output_v3/`、
  `training_output_v4/`：模型训练和转换历史产物；以各目录报告和 `models/` 文件为准。
- `README.md`：迁移协议、模型比较、Flash/SRAM、Keil 接入和已知差异说明。

当前迁移保留 SCC8660 QQVGA 160x120 RGB565、LPUART5 115200、`A5 5A` 请求协议、
`0xFE` 数字分类、`0xBB` box 分类和原类别字节响应。模型加入
`project/user/models` 后，还要把 `classification_model_data.s` 加入汇编组。

`MCXVision/classification` 是 MCX 部署工程的唯一真源。修改代码或模型后直接在
`project/mdk/mcx_vision_board.uvprojx` 中 Rebuild；不要在 `tmp/` 建立第二份工程。
`Objects/`、`Listings/`、map 和 build log 都是本地生成物，不作为源码提交。

## NeutronScratch 验证方法

`NeutronScratch` 是 Neutron 转换后 TFLite FlatBuffer 中的运行时 scratch 工作区，
不是 `.tflite` 文件大小、所有 tensor 大小之和，也不是完整 Tensor Arena。计算必须使用
运行时形状和元素 dtype 宽度：

```text
runtime_bytes = product(shape) * bytes_per_element(dtype)
```

`model_train/train_model/inspect_neutron.py` 使用 Python `tflite` FlatBuffer 解析器：

1. 读取 Neutron 转换后的 `.tflite`，遍历第一个 subgraph 的 tensors。
2. 收集名称以 `Neutron` 开头的 tensor，定位 `NeutronScratch`。
3. 用 shape 元素数乘 `TYPE_WIDTH[dtype]` 得到 `neutron_scratch_bytes`。
4. 另算模型文件大小、输入运行时字节数、`scratch_plus_input_bytes` 和 tensor 明细。
5. 用 `--max-scratch`、`--max-model-bytes` 做门禁，默认分别为 `100000` 和 `430000`
   字节；失败时退出码为 2。

复现命令：

```bash
python model_train/train_model/inspect_neutron.py \
  MCXVision/classification/project/user/models/box_student_npu.tflite \
  --max-scratch 100000 \
  --max-model-bytes 430000 \
  --report tmp/box_cls_neutron.json
```

`model_train/train_model/convert_neutron.py` 先调用 Neutron converter 的
`--target mcxn94x --run-after-generate`，再调用同一套 `inspect_model()` 检查门槛。
`--run-after-generate` 不能替代目标板上的真实推理验证。

### 已验证数值

- 早期 box 模型曾测得 `NeutronScratch = 576000` bytes；即使模型文件能放在 SD 卡，
  scratch 仍超过 MCXN947 可用 SRAM，因此不能部署。
- 当前 MCX 专用 box 模型的 baseline scratch 约 `86400` bytes，低于默认 `100000` 门槛；
  仍要同时检查转换后模型不超过 `430000` bytes，并结合 Tensor Arena、帧缓存和其它
  工作区完成实机内存验证。
- `NeutronScratch`、动态 tensor 总量和 `.tflite` 文件大小是不同指标，报告中必须注明
  指标名称和测量脚本，不能用文件大小推断 scratch。

## 变更边界

- 模型可存放在 SD 卡，但 TFLite Micro/Neutron 推理需要模型位于连续、可随机访问的
  内存；SD 卡存储不会自动解决 scratch 或 Tensor Arena 超限。
- `classification_model_data.s` 当前通过 `.incbin` 将模型编译进片内 Flash；替换模型
  后必须同步替换 `project/user/models`、汇编引用和 Neutron 报告。
- 修改模型结构、输入尺寸、dtype 或 converter 参数后，必须重新执行 Neutron 转换和
  `inspect_neutron.py`，不能沿用旧报告中的 scratch 数值。
- 静态 FlatBuffer 检查通过不代表 Keil 链接、Flash 分区、Tensor Arena、摄像头 DMA
  或一次真实分类一定可用，最终必须在 MCX 目标板验证。
