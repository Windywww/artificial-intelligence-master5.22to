#include "classification_core.h"
#include "classification_models.h"

#if defined(__cplusplus)
extern "C"
{
#endif
#include "zf_common_headfile.h"
#if defined(__cplusplus)
}
#endif

#include <string.h>

#if (SCC8660_W != 160) || (SCC8660_H != 120)
#error "classification requires FRAME_SIZE == SCC8660_QQVGA"
#endif

namespace
{

static const uint8_t kGoalFlag = 0xFE;
static const uint8_t kBoxFlag = 0xBB;
static const uint8_t kUnknownClass = 11;
static const float kDigitProbabilityThreshold = 0.70f;
static const float kBoxProbabilityThreshold = 0.50f;
static const size_t kMaxDigitBlobs = 12;

uint16_t s_stable_frame[mcxvision::kImageWidth * mcxvision::kImageHeight];
mcxvision::UartPacketParser s_uart_parser;

void send_class(uint8_t value)
{
    user_uart_putchar(static_cast<char>(value));
}

int poll_command(void)
{
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

void process_goal(const uint16_t *frame)
{
    mcxvision::Blob purple_blob;
    if(mcxvision::find_blobs(frame, mcxvision::kPurpleThreshold,
                             mcxvision::kCenterRoi, 800U, 0xFFFFFFFFU,
                             false, &purple_blob, 1U) != 0U)
    {
        send_class(0);
        return;
    }

    const mcxvision::Roi full_frame = {0, 0, mcxvision::kImageWidth, mcxvision::kImageHeight};
    mcxvision::Blob blobs[kMaxDigitBlobs];
    const size_t blob_count = mcxvision::find_blobs(
        frame, mcxvision::kBlackThreshold, full_frame,
        1500U, 9500U, true, blobs, kMaxDigitBlobs);
    if(blob_count == 0U || !mcxvision::model_select(mcxvision::kDigitModel))
    {
        send_class(kUnknownClass);
        return;
    }

    int best_label = -1;
    float best_probability = 0.0f;
    uint8_t canvas[mcxvision::kDigitCanvasSize * mcxvision::kDigitCanvasSize];
    for(size_t i = 0; i < blob_count; ++i)
    {
        mcxvision::make_digit_canvas(frame, blobs[i], canvas);
        int label = -1;
        float probability = 0.0f;
        if(mcxvision::model_fill_digit(canvas)
            && mcxvision::model_run()
            && mcxvision::model_top1(&label, &probability)
            && probability > kDigitProbabilityThreshold
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
        send_class(static_cast<uint8_t>(best_label + 1));
    }
    else
    {
        send_class(kUnknownClass);
    }
}

void process_box(const uint16_t *frame)
{
    int label = -1;
    float probability = 0.0f;
    if(!mcxvision::model_select(mcxvision::kBoxModel)
        || !mcxvision::model_fill_box(frame, mcxvision::kCenterRoi)
        || !mcxvision::model_run()
        || !mcxvision::model_top1(&label, &probability)
        || label < 0 || label >= 10
        || probability < kBoxProbabilityThreshold)
    {
        send_class(kUnknownClass);
        return;
    }

    zf_debug_printf("box:%d p:%d\r\n", label,
                    static_cast<int>(probability * 100.0f));
    send_class(static_cast<uint8_t>(label + 1));
}

} // namespace

extern "C" void ezh_copy_slice_to_model_input(uint32_t idx,
                                                uint32_t cam_slice_buffer,
                                                uint32_t cam_slice_width,
                                                uint32_t cam_slice_height,
                                                uint32_t max_idx)
{
    (void)idx;
    (void)cam_slice_buffer;
    (void)cam_slice_width;
    (void)cam_slice_height;
    (void)max_idx;
}

extern "C" int main(void)
{
    zf_board_init();
    user_uart_init();

    // The stock IRQ consumes received bytes without forwarding them. Poll RX instead.
    LPUART_DisableInterrupts(USER_USART, kLPUART_RxDataRegFullInterruptEnable);
    DisableIRQ(LP_FLEXCOMM5_IRQn);

    system_delay_ms(300);
    scc8660_init();
    scc8660_set_brightness(950);
    scc8660_set_white_balance(0x5d, 0x40, 0x5e);
    system_delay_ms(800);

    zf_debug_printf("classification_ready\r\n");
    int pending_flag = -1;
    while(1)
    {
        const int received_flag = poll_command();
        if(received_flag >= 0 && pending_flag < 0)
        {
            pending_flag = received_flag;
            zf_debug_printf("flag:%d\r\n", pending_flag);
        }

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
            pending_flag = -1;
        }
    }
}
