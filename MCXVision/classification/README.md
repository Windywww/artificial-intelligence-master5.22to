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

| 文件 | 大小 | 输入 | Neutron scratch | 说明 |
| --- | ---: | --- | ---: | --- |
| `num_cls_npu.tflite` | 28,032 B | 28x28 灰度 | 约 16.5 KB 动态张量 | 数字分类 |
| `box_student_npu.tflite` | 400,992 B | 120x120 RGB INT8 | 86,400 B | box 分类 |

部署用 student 模型的 SHA-256 为
`6F0A8F6CD61A0AF426430BBE150CD0C1D7E8B153BA2734C9EED03F7DE6CD677F`，
输入量化参数为 `scale=1.0, zero_point=-128`，输出为 10 类。其 INT8 固定验证集
准确率为 97.05%，真实采集测试集准确率为 99.79%；这些是离线训练指标，仍需通过
本页后述的 MCX 目标板验收。完整静态门禁结果见
`models/box_student_npu.neutron.json`。

## Flash 和 SD 卡

MCXN947 物理片内 Flash 为 2 MB。官方例程的默认 scatter 文件给 Core0 普通代码和只读数据的上限约为 `0xBFC00`，即 767 KB；程序、常量和嵌入模型共同占用该区域。

当前两个部署模型合计 429,024 B，加上约 95 KB 程序和汇编对齐填充，预计仍低于默认区域上限；最终以新构建生成的 `.map` 为准。`classification_model_data.s` 使用 `.incbin` 把模型放入片内 Flash，不占用模型文件大小等量的 SRAM。

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

本机集成快照可在 `tmp/mcx_build/project/mdk` 下执行全量 Rebuild：

```powershell
& 'D:\Apps\work\Keil5\UV4\UV4.exe' -r '.\mcx_vision_board.uvprojx' `
  -j0 -o '.\student_rebuild.log'
```

当前机器安装的是未激活的 MDK-Lite。2026-07-19 使用新 student 模型执行 Rebuild，
ArmClang 编译和链接布局均成功，最终 AXF 仍被授权限制 `L6050U` 拒绝（代码
95,200 B）；按说明书激活 Keil 后才能完成链接。这不是芯片 Flash 或 SRAM 容量错误。

该次链接器生成的 map 已确认：

- `box_model_data_end - box_model_data = 400,992 B`，嵌入的是 student 模型。
- `ER_m_text = 609,900 / 785,408 B`。
- `LR_m_text = 715,008 / 786,432 B`，距离区域上限 71,424 B。
- `RW_m_data = 333,472 / 416,756 B`，静态余量 83,284 B。

以上 map 数值不能替代目标板上的 `AllocateTensors()`、一次真实 `Invoke()` 和
`model_arena_used()` 检查；激活 MDK 后必须重新 Rebuild 并以新 map 为准。

## 内存设计

- 两个模型共用一个 190 KB Tensor Arena，切换 `0xFE`/`0xBB` 时重建解释器。
- box 部署模型包含 86.4 KB `NeutronScratch`，scratch 加输入张量为 129.6 KB。
- 收到命令后先复制完整帧，避免摄像头 DMA 在推理时覆盖图像。
- 连通域工作区约 57.6 KB，摄像头原始帧和稳定帧各约 38.4 KB。

第一次切换模型会执行 `AllocateTensors()`，延迟高于连续执行同一种分类，但响应协议不变。

## 不能完全一致的部分与建议

1. **box 精度**：student 模型离线 INT8 准确率为 97.05%，但 SCC8660 的现场图像分布可能不同。应在比赛光照下与 OpenART 使用同一套目标做逐类验收；不能仅用离线指标代替实机结果。
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

部署前可在包含 Python `tflite` 解析器的训练环境中复核实际复制文件：

```powershell
python model_train/train_model/inspect_neutron.py `
  MCXVision/classification/project/user/models/box_student_npu.tflite `
  --max-scratch 100000 --max-model-bytes 430000
```

模型由 eIQ 1.10.0 转换：

```powershell
& 'D:\Apps\work\eIQ_Toolkit_v1.10.0\bin\neutron-converter\v1.2.0\neutron-converter.exe' `
  --input box_student_int8.tflite --output box_student_npu.tflite `
  --target mcxn94x --run-after-generate
```

数字和 box 转换结果均已通过转换器 dummy inference，结构为 `NEUTRON_GRAPH + SLICE + RESHAPE + SOFTMAX`。

目标板的命令口为 LPUART5 115200 8N1，驱动配置使用 P1_16/P1_17；USB-TTL 与板端
TX/RX 交叉连接并共地。调试日志由独立的 LPUART4 输出。安装 `pyserial` 后可运行
串口验收工具；工具严格等待当前响应后才发送下一条命令：

```powershell
python MCXVision/classification/tools/board_serial_test.py --port COM7 request --flag box --count 10
python MCXVision/classification/tools/board_serial_test.py --port COM7 protocol --flag box
python MCXVision/classification/tools/board_serial_test.py --port COM7 stress --requests 200
```

`0` 表示紫色空类，`1..10` 对应模型标签 `0..9`，`11` 表示未知或推理失败。

## 目标板验收

1. 下载后手动复位，在 LPUART4 确认 `classification_ready`。
2. 首次发送 `A5 5A BB BB`，确认日志包含 `box:arena:<bytes>`、
   `box_raw:label:<n> p:<percent>`，且 arena 小于 194,560 B。
3. 连续发送同一 flag 测量首次加载与同模型 P50/P95，再用 `stress` 测量模型切换
   P50/P95；200 个交替请求必须零超时、零复位、零 HardFault。
4. box 十类各采集 10 次，现场准确率至少 95%，且不比同场 OpenART 低超过 3 个
   百分点。
5. goal 十类各采集 5 次，另测 10 次紫色空类和 10 次无有效黑色区域；goal 准确率
   不比 OpenART 低超过 3 个百分点，两个特殊分支必须全部正确。
6. 用非对称目标确认软件 180 度方向与 OpenART 一致，并确认 box 目标处于
   `(20, 0, 120, 120)` ROI。

若 LPUART5 返回 `11`，先查看 LPUART4：`fail:select` 表示模型或 Tensor Arena 初始化
失败，`fail:fill` 表示输入形状或 dtype 不兼容，`fail:invoke` 表示 Neutron 推理失败，
`fail:top1` 表示输出 tensor 不符合预期；`box:reject` 则只是类别或置信度未过门槛。
