# OpenART classification.py 到 MCXVision 的迁移

本目录把 `openART/classification.py` 的有效行为迁移到逐飞 MCXVision 官方 C/C++ 库，并提供已经转换为 MCXN94x Neutron 格式的模型。

## 保留的行为

- SCC8660 QQVGA（160x120）RGB565 图像输入。
- 用户串口 LPUART5，115200 baud。
- 接收 `A5 5A flag flag`，返回一个原始类别字节。
- `0xFE`：紫色空类别判断、黑色连通域提取、28x28 数字分类。
- `0xBB`：中心 `(20, 0, 120, 120)` ROI 的 box 分类。
- 数字阈值 `0.70`、box 阈值 `0.50`、未知响应 `11`。
- 软件执行水平镜像和垂直翻转，等价于原脚本的 180 度旋转。

请求和响应示例：

```text
请求: A5 5A FE FE  -> goal 分类
请求: A5 5A BB BB  -> box 分类
响应: 单字节 00..0A；失败或低置信度为 0B
```

## 模型选择

所有模型位于 `project/user/models`。

| 文件 | 大小 | 运行时动态张量 | 说明 |
| --- | ---: | ---: | --- |
| `num_cls_npu.tflite` | 28,032 B | 约 16.5 KB | 实际部署的数字模型 |
| `box_cls_npu.tflite` | 400,736 B | 约 129.6 KB | 实际部署的 MCX 专用 box 模型 |
| `box_cls_openart.tflite` | 106,832 B | - | 原 OpenART `box2.2.5` 模型 |
| `box_cls_openart_npu_oversized.tflite` | 100,192 B | 约 619 KB | 仅供分析，不可部署 |

原 box 模型转换后需要 `576,000` 字节 `NeutronScratch`，单是 scratch 就超过 MCXN947 的 512 KB 物理 SRAM，因此模型文件即使放在 SD 卡也不能运行。

实际部署版重新训练为 120x120 RGB、10 类、全 INT8 模型，接口和类别编号不变。994 张固定随机验证集上的量化结果：

| 模型 | 正确数 | 准确率 |
| --- | ---: | ---: |
| 原 `box2.2.5` | 962/994 | 96.78% |
| MCX 专用模型 | 922/994 | 92.76% |

详细混淆矩阵见 `models/box_model_comparison.json`，训练摘要见 `models/box_cls_mcx_report.json`。可用 `tools/train_box_mcx.py` 重新训练，用 `tools/evaluate_box_tflite.py` 复现量化验证。

## Flash 和 SD 卡

MCXN947 物理片内 Flash 为 2 MB。官方例程的默认 scatter 文件给 Core0 普通代码和只读数据的上限约为 `0xBFC00`，即 767 KB；程序、常量和嵌入模型共同占用该区域。

当前两个部署模型合计约 419 KB，加上约 95 KB 程序，仍低于默认区域上限。`classification_model_data.s` 使用 `.incbin` 把模型放入片内 Flash，不占用模型文件大小等量的 SRAM。

模型可以存储在 SD 卡，但 TensorFlow Lite Micro 不能直接从 SD 文件流推理。`tflite::GetModel()` 要求模型在推理期间位于连续、可随机访问的内存中：

- SD -> SRAM：实现简单，但会额外占用完整模型大小的 SRAM，本项目不适合。
- SD -> 预留片内 Flash：可实现现场更新且不长期占 SRAM，但需增加 Flash 分区、擦写、校验和掉电保护。
- 编译进片内 Flash：本迁移采用的方式，也是官方模型例程的方式。

如果以后需要 SD 更新，建议采用“启动时校验 SD 模型并写入预留 Flash 槽，运行时从 Flash 建立模型”的方案，而不是从 SD 读入 SRAM。

## 接入官方工程

以官方 `【例程】Example/E07_mcx_vision_camera_qqvga_demo` 为底板：

1. 将本目录 `project/user` 的内容合并到例程 `project/user`，用本目录的 `main.cpp` 替换原文件。
2. 在 `libraries/zf_devices/zf_device_scc8660.h` 中把 `FRAME_SIZE` 改成 `SCC8660_QQVGA`。尺寸不正确时迁移代码会直接编译报错。
3. 从 Keil 构建中排除官方的 `zf_model_process.cpp`、`model.cpp`、`model_data.s` 和 `model_ops_npu.cpp`，避免重复占用 Tensor Arena。
4. 把 `classification_core.cpp`、`classification_models.cpp`、`classification_model_ops_npu.cpp` 作为 C++ 文件加入 `user` 组。
5. 把 `classification_model_data.s` 作为汇编文件加入；两个头文件可作为普通头文件加入。
6. 保持 `models` 子目录位于 `project/user/models`。汇编文件的 `.incbin` 路径以 `project/mdk` 为工作目录。
7. 使用 MDK 5.38a 或更高版本构建、下载，下载后手动复位。

当前机器安装的是未激活的 MDK-Lite。实际集成工程已经通过 ArmClang 编译并解决全部符号问题，但最终链接被授权限制 `L6050U` 拒绝（代码约 94,712 B）；按说明书激活 Keil 后才能完成链接。这不是芯片 Flash 容量错误。

## 内存设计

- 两个模型共用一个 190 KB Tensor Arena，切换 `0xFE`/`0xBB` 时重建解释器。
- box 部署模型包含 86.4 KB `NeutronScratch`，全部动态张量约 129.6 KB。
- 收到命令后先复制完整帧，避免摄像头 DMA 在推理时覆盖图像。
- 连通域工作区约 57.6 KB，摄像头原始帧和稳定帧各约 38.4 KB。

第一次切换模型会执行 `AllocateTensors()`，延迟高于连续执行同一种分类，但响应协议不变。

## 不能完全一致的部分与建议

1. **box 精度**：MCX 专用模型比原模型低约 4.0 个百分点。建议增加易混类别 03/04/08/09 的现场样本，保持轻量网络结构重新训练；不能直接换回原模型，因为其 SRAM 超限。
2. **摄像头参数**：SCC8660 公共接口没有 OpenART 的 `gain_db=2.0`；`exposure_us=950` 也只能用 brightness/exposure 数值近似。代码设置 brightness 950、手动白平衡 `0x5d/0x40/0x5e`，应在比赛光照下重新标定。
3. **颜色阈值**：OpenART 使用内部 LAB 查表，迁移版使用标准 D65 CIELAB 快速近似。原阈值已保留，但边界像素可能不同，应使用现场图像微调 `kBlackThreshold` 和 `kPurpleThreshold`。
4. **缩放**：迁移版使用最近邻缩放，OpenART `draw_image` 的内部取样细节未公开。若数字临界样本下降，优先用迁移版预处理重新生成训练集，或改为双线性插值。
5. **调试显示**：原脚本的矩形和文字只用于 IDE 预览，不影响串口结果。迁移版默认不启用 IPS200，以保留帧率。

## 验证命令

纯图像核心不依赖 MCX SDK，可在 Windows 上验证串口状态机、方向映射、LAB 连通域和 28x28 画布：

```powershell
g++ -std=c++11 -O2 `
  MCXVision/classification/project/user/classification_core.cpp `
  MCXVision/classification/tests/classification_core_test.cpp `
  -o MCXVision/classification/tests/classification_core_test.exe
MCXVision/classification/tests/classification_core_test.exe
```

模型由本机 eIQ 1.10.0 转换：

```powershell
& 'D:\Apps\work\eIQ_Toolkit_v1.10.0\bin\neutron-converter\v1.2.0\neutron-converter.exe' `
  --input box_cls.tflite --output box_cls_npu.tflite `
  --target mcxn94x --run-after-generate
```

数字和 box 转换结果均已通过转换器 dummy inference，结构为 `NEUTRON_GRAPH + SLICE + RESHAPE + SOFTMAX`。
