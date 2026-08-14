#include "classification_core.h"
#include "classification_models.h"

/*
 * MCXN947 板端主流程：
 * LPUART5 收到 A5 5A flag flag
 *   -> 等待 SCC8660 完整帧 -> 复制稳定帧 -> goal/box 分类
 *   -> LPUART5 返回一个原始类别字节。
 * LPUART4 的 zf_debug_printf 只输出诊断日志，不参与单字节响应协议。
 */

#if defined(__cplusplus)
extern "C"
{
#endif
// 逐飞库按 C 编译；extern "C" 防止 C++ 名字改编，保证 C 符号可链接。
#include "zf_common_headfile.h"
#if defined(__cplusplus)
}
#endif

#include <string.h>

#if (SCC8660_W != 160) || (SCC8660_H != 120)
// 后续代码按 160x120 索引内存，尺寸不符时直接在编译期拒绝。
#error "classification requires FRAME_SIZE == SCC8660_QQVGA"
#endif

namespace
{

// 命令和响应值保持与原 OpenART classification.py 一致。
static const uint8_t kGoalFlag = 0xFE;
static const uint8_t kBoxFlag = 0xBB;
static const uint8_t kUnknownClass = 11;
// 低于阈值不强行分类，统一返回未知类别 11。
static const float kDigitProbabilityThreshold = 0.70f;
static const float kBoxProbabilityThreshold = 0.50f;
static const size_t kMaxDigitBlobs = 12;

// 摄像头 DMA 写 g_camera_buffer；推理使用副本，防止下一帧覆盖正在处理的图像。
uint16_t s_stable_frame[mcxvision::kImageWidth * mcxvision::kImageHeight];
mcxvision::UartPacketParser s_uart_parser;

void send_class(uint8_t value)
{
    // 发送原始二进制字节，不发送字符 '0'..'9'，也不附加换行。
    user_uart_putchar(static_cast<char>(value));
}

int poll_command(void)
{
    // 非阻塞取走当前已到达 LPUART5 FIFO 的字节；没有完整包时返回 -1。
    uint8_t flag = 0;
    while((LPUART_GetStatusFlags(USER_USART) & kLPUART_RxDataRegFullFlag) != 0U)
    {
        if(s_uart_parser.push(LPUART_ReadByte(USER_USART), &flag))
        {
            return flag;
        }
    }
    return -1;
}

void log_model_ready(const char *model)
{
    // arena_used 是目标板实际分配量，用于内存门禁。
    zf_debug_printf("%s:arena:%u\r\n", model,
                    static_cast<unsigned int>(mcxvision::model_arena_used()));
}

void log_model_failure(const char *model, const char *stage)
{
    // 分开记录 select/fill/invoke/top1，便于定位模型、输入、NPU 或输出错误。
    zf_debug_printf("%s:fail:%s\r\n", model, stage);
}

void process_goal(const uint16_t *frame)
{
    // 原 OpenART 逻辑优先识别紫色空类别，命中后直接返回 0。
    mcxvision::Blob purple_blob;
    if(mcxvision::find_blobs(frame, mcxvision::kPurpleThreshold,
                             mcxvision::kCenterRoi, 800U, 0xFFFFFFFFU,
                             false, &purple_blob, 1U) != 0U)
    {
        zf_debug_printf("goal:purple\r\n");
        send_class(0);
        return;
    }

    // 没有紫色后，在全帧寻找面积合适的黑色数字候选区域。
    const mcxvision::Roi full_frame = {0, 0, mcxvision::kImageWidth, mcxvision::kImageHeight};
    mcxvision::Blob blobs[kMaxDigitBlobs];
    const size_t blob_count = mcxvision::find_blobs(
        frame, mcxvision::kBlackThreshold, full_frame,
        1500U, 9500U, true, blobs, kMaxDigitBlobs);
    if(blob_count == 0U)
    {
        zf_debug_printf("digit:reject:no_blob\r\n");
        send_class(kUnknownClass);
        return;
    }
    // 首次请求或从 box 切换回来时，这里会重建 TFLite 解释器。
    if(!mcxvision::model_select(mcxvision::kDigitModel))
    {
        log_model_failure("digit", "select");
        send_class(kUnknownClass);
        return;
    }
    log_model_ready("digit");

    // 一帧可有多个候选区域，最终取超过阈值且概率最高的一个。
    int best_label = -1;
    float best_probability = 0.0f;
    uint8_t canvas[mcxvision::kDigitCanvasSize * mcxvision::kDigitCanvasSize];
    for(size_t i = 0; i < blob_count; ++i)
    {
        mcxvision::make_digit_canvas(frame, blobs[i], canvas);
        int label = -1;
        float probability = 0.0f;
        if(!mcxvision::model_fill_digit(canvas))
        {
            log_model_failure("digit", "fill");
            continue;
        }
        if(!mcxvision::model_run())
        {
            log_model_failure("digit", "invoke");
            continue;
        }
        if(!mcxvision::model_top1(&label, &probability))
        {
            log_model_failure("digit", "top1");
            continue;
        }

        zf_debug_printf("digit_raw:blob:%u label:%d p:%d\r\n",
                        static_cast<unsigned int>(i), label,
                        static_cast<int>(probability * 100.0f));
        if(probability > kDigitProbabilityThreshold
            && probability > best_probability)
        {
            best_label = label;
            best_probability = probability;
        }
    }

    if(best_label >= 0 && best_label < 10)
    {
        zf_debug_printf("digit:%d p:%d\r\n", best_label,
                        static_cast<int>(best_probability * 100.0f));
        // 协议响应 1..10 对应模型标签 0..9，因此这里加 1。
        send_class(static_cast<uint8_t>(best_label + 1));
    }
    else
    {
        send_class(kUnknownClass);
    }
}

void process_box(const uint16_t *frame)
{
    // box 分支固定把中心 120x120 ROI 送入十分类模型，不做颜色连通域预筛选。
    int label = -1;
    float probability = 0.0f;
    if(!mcxvision::model_select(mcxvision::kBoxModel))
    {
        log_model_failure("box", "select");
        send_class(kUnknownClass);
        return;
    }
    log_model_ready("box");

    if(!mcxvision::model_fill_box(frame, mcxvision::kCenterRoi))
    {
        log_model_failure("box", "fill");
        send_class(kUnknownClass);
        return;
    }
    if(!mcxvision::model_run())
    {
        log_model_failure("box", "invoke");
        send_class(kUnknownClass);
        return;
    }
    if(!mcxvision::model_top1(&label, &probability))
    {
        log_model_failure("box", "top1");
        send_class(kUnknownClass);
        return;
    }

    zf_debug_printf("box_raw:label:%d p:%d\r\n", label,
                    static_cast<int>(probability * 100.0f));
    // 同时校验标签范围和置信度，避免异常输出变成有效协议值。
    if(label < 0 || label >= 10 || probability < kBoxProbabilityThreshold)
    {
        zf_debug_printf("box:reject\r\n");
        send_class(kUnknownClass);
        return;
    }

    zf_debug_printf("box:%d p:%d\r\n", label,
                    static_cast<int>(probability * 100.0f));
    // 模型标签 0..9 映射成协议响应 1..10。
    send_class(static_cast<uint8_t>(label + 1));
}

} // namespace

extern "C" void ezh_copy_slice_to_model_input(uint32_t idx,
                                                uint32_t cam_slice_buffer,
                                                uint32_t cam_slice_width,
                                                uint32_t cam_slice_height,
                                                uint32_t max_idx)
{
    /*
     * 逐飞官方旧模型流程会引用这个 C 回调把摄像头切片送入模型。
     * 本迁移复制完整帧后再预处理，所以保留符号供链接，但无需处理这些参数。
     */
    (void)idx;
    (void)cam_slice_buffer;
    (void)cam_slice_width;
    (void)cam_slice_height;
    (void)max_idx;
}

extern "C" int main(void)
{
    // 初始化时钟、引脚和调试串口，再初始化用户串口 LPUART5。
    zf_board_init();
    user_uart_init();

    // 官方接收中断会取走字节却不转交解析器，因此关闭中断，改由主循环轮询 RX。
    LPUART_DisableInterrupts(USER_USART, kLPUART_RxDataRegFullInterruptEnable);
    DisableIRQ(LP_FLEXCOMM5_IRQn);

    // 给电源和传感器留出稳定时间，再设置接近 OpenART 的亮度与白平衡。
    system_delay_ms(300);
    scc8660_init();
    scc8660_set_brightness(950);
    scc8660_set_white_balance(0x5d, 0x40, 0x5e);
    system_delay_ms(800);

    // ready 只输出到调试串口，表示初始化完成并开始接收命令。
    zf_debug_printf("classification_ready\r\n");
    // 同一时间只保存一个请求，当前请求处理完成前不会覆盖 pending_flag。
    int pending_flag = -1;
    while(1)
    {
        const int received_flag = poll_command();
        if(received_flag >= 0 && pending_flag < 0)
        {
            pending_flag = received_flag;
            zf_debug_printf("flag:%d\r\n", pending_flag);
        }

        // scc8660_finish 表示 DMA 写完一整帧，此时复制才不会拿到半帧图像。
        if(pending_flag >= 0 && scc8660_finish)
        {
            memcpy(s_stable_frame, (const void *)g_camera_buffer,
                   sizeof(s_stable_frame));
            scc8660_finish = 0;

            if(pending_flag == kGoalFlag)
            {
                process_goal(s_stable_frame);
            }
            else if(pending_flag == kBoxFlag)
            {
                process_box(s_stable_frame);
            }
            // 无论 flag 是否受支持，处理一次后都释放槽位，继续等待下一条命令。
            pending_flag = -1;
        }
    }
}
