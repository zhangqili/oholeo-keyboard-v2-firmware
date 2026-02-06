#include "adc.h"

#include <stdio.h>
#include "board.h"
#include "analog.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_debug_console.h"

#include "hpm_adc16_drv.h"

#if defined(HPMSOC_HAS_HPMSDK_PWM)
#include "hpm_pwm_drv.h"
#endif

#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#include "hpm_pwmv2_drv.h"
#endif

#ifndef ADC_SOC_NO_HW_TRIG_SRC
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#endif

#ifndef APP_ADC16_CORE
#define APP_ADC16_CORE BOARD_RUNNING_CORE
#endif

#define APP_ADC16_CLOCK_BUS  BOARD_APP_ADC16_CLK_BUS
#define __ADC16_USE_SW_TRIG
#define APP_ADC16_CH_SAMPLE_CYCLE            (21U)
#define APP_ADC16_CH_WDOG_EVENT              (1 << BOARD_APP_ADC16_CH_1)

#define APP_ADC16_SEQ_START_POS              (0U)
#define APP_ADC16_SEQ_DMA_BUFF_LEN_IN_4BYTES (2048U)
#define APP_ADC16_SEQ_IRQ_EVENT              adc16_event_seq_full_complete

#ifndef ADC_SOC_NO_HW_TRIG_SRC
#define APP_ADC16_HW_TRIG_SRC_PWM_REFCH_A    (8U)
#define APP_ADC16_HW_TRIG_SRC                BOARD_APP_ADC16_HW_TRIG_SRC
#define APP_ADC16_HW_TRGM                    BOARD_APP_ADC16_HW_TRGM
#define APP_ADC16_HW_TRGM_IN                 BOARD_APP_ADC16_HW_TRGM_IN
#define APP_ADC16_HW_TRGM_OUT_SEQ            BOARD_APP_ADC16_HW_TRGM_OUT_SEQ
#define APP_ADC16_HW_TRGM_OUT_PMT            BOARD_APP_ADC16_HW_TRGM_OUT_PMT
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#define APP_ADC16_HW_TRGM_SRC_OUT_CH         (0U)
#endif
#endif

#ifndef APP_ADC16_TRIG_SRC_FREQUENCY
#define APP_ADC16_TRIG_SRC_FREQUENCY         (20000U)
#endif

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc0_seq_buff[APP_ADC16_SEQ_DMA_BUFF_LEN_IN_4BYTES];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc1_seq_buff[APP_ADC16_SEQ_DMA_BUFF_LEN_IN_4BYTES];

#ifndef APP_SAMPLE_LENGTH
#define APP_SAMPLE_LENGTH 16
#endif



uint8_t seq_adc_channel0[] = {1,2,3,4,5};
uint8_t seq_adc_channel1[] = {6,7,11,14,15};
uint8_t current_cycle_bit;

__IO uint8_t seq_complete_flag;
__IO uint8_t trig_complete_flag;
__IO uint32_t res_out_of_thr_flag;

hpm_stat_t process_seq_data(uint32_t *buff, int32_t start_pos, uint32_t len)
{
    adc16_seq_dma_data_t *dma_data = (adc16_seq_dma_data_t *)buff;

    if (ADC16_IS_SEQ_DMA_BUFF_LEN_INVLAID(len)) {
        return status_invalid_argument;
    }

    current_cycle_bit = !current_cycle_bit;
    
    printf("%02d\t",  current_cycle_bit);
    for (uint32_t i = start_pos; i < start_pos + len; i++) {
        printf("%04d\t", dma_data[i].result);

        //if (dma_data[i].cycle_bit != current_cycle_bit) {
        //    printf("Error: Cycle bit is not expected value[%d]!\n", current_cycle_bit);
        //    while (1) {
        //
        //    }
        //}
    }
    printf("\n");

    return status_success;
}

#ifndef ADC_SOC_NO_HW_TRIG_SRC

#if defined(HPMSOC_HAS_HPMSDK_PWM)
void init_trigger_source(PWM_Type *ptr)
{
    pwm_cmp_config_t pwm_cmp_cfg;
    pwm_output_channel_t pwm_output_ch_cfg;

    int mot_clock_freq;

    mot_clock_freq = clock_get_frequency(BOARD_APP_ADC16_HW_TRIG_SRC_CLK_NAME);

    /* reload value */
    pwm_set_reload(ptr, 0, (mot_clock_freq/APP_ADC16_TRIG_SRC_FREQUENCY) - 1);

    /* Set a comparator */
    memset(&pwm_cmp_cfg, 0x00, sizeof(pwm_cmp_config_t));
    pwm_cmp_cfg.enable_ex_cmp  = false;
    pwm_cmp_cfg.mode           = pwm_cmp_mode_output_compare;
    pwm_cmp_cfg.update_trigger = pwm_shadow_register_update_on_shlk;

    /* Select comp8 and trigger at the middle of a pwm cycle */
    pwm_cmp_cfg.cmp = ((mot_clock_freq/APP_ADC16_TRIG_SRC_FREQUENCY) - 1) >> 1;
    pwm_config_cmp(ptr, APP_ADC16_HW_TRIG_SRC_PWM_REFCH_A, &pwm_cmp_cfg);

    /* Issue a shadow lock */
    pwm_issue_shadow_register_lock_event(APP_ADC16_HW_TRIG_SRC);

    /* Set comparator channel to generate a trigger signal */
    pwm_output_ch_cfg.cmp_start_index = APP_ADC16_HW_TRIG_SRC_PWM_REFCH_A;   /* start channel */
    pwm_output_ch_cfg.cmp_end_index   = APP_ADC16_HW_TRIG_SRC_PWM_REFCH_A;   /* end channel */
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(ptr, APP_ADC16_HW_TRIG_SRC_PWM_REFCH_A, &pwm_output_ch_cfg);

    /* Start the comparator counter */
    pwm_start_counter(ptr);
}

void stop_trigger_source(PWM_Type *ptr)
{
    pwm_stop_counter(ptr);
}

void start_trigger_source(PWM_Type *ptr)
{
    pwm_start_counter(ptr);
}
#endif

void init_trigger_mux(TRGM_Type *ptr, uint8_t input, uint8_t output)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type = trgm_output_same_as_input;

    trgm_output_cfg.input  = input;
    trgm_output_config(ptr, output, &trgm_output_cfg);
}
#endif

hpm_stat_t init_common_config(void)
{
    {
        adc16_config_t cfg;

        /* initialize an ADC instance */
        adc16_get_default_config(&cfg);

        cfg.res            = adc16_res_16_bits;
        cfg.conv_mode      = adc16_conv_mode_sequence;
        cfg.adc_clk_div    = adc16_clock_divider_4;
        cfg.sel_sync_ahb   = (APP_ADC16_CLOCK_BUS == clock_get_source(clock_adc0)) ? true : false;
        cfg.adc_ahb_en = true;

        /* adc16 initialization */
        if (adc16_init(HPM_ADC0, &cfg) == status_success) {
            /* enable irq */
            //intc_m_enable_irq_with_priority(IRQn_ADC0, 1);
            //return status_success;
        } else {
            printf("%s initialization failed!\n", "ADC0");
            return status_fail;
        }
    }
    {
        adc16_config_t cfg;

        /* initialize an ADC instance */
        adc16_get_default_config(&cfg);

        cfg.res            = adc16_res_16_bits;
        cfg.conv_mode      = adc16_conv_mode_sequence;
        cfg.adc_clk_div    = adc16_clock_divider_4;
        cfg.sel_sync_ahb   = (APP_ADC16_CLOCK_BUS == clock_get_source(clock_adc1)) ? true : false;
        cfg.adc_ahb_en = true;

        /* adc16 initialization */
        if (adc16_init(HPM_ADC1, &cfg) == status_success) {
            /* enable irq */
            //intc_m_enable_irq_with_priority(IRQn_ADC1, 1);
            return status_success;
        } else {
            printf("%s initialization failed!\n", "ADC1");
            return status_fail;
        }
    }
}

void channel_result_out_of_threshold_handler(void)
{
    //adc16_channel_threshold_t threshold;
    //uint32_t i = 31;

    //if (res_out_of_thr_flag) {
    //    while (i--) {
    //        if ((res_out_of_thr_flag >> i) & 0x01) {
    //            adc16_get_channel_threshold(HPM_ADC0, i, &threshold);
    //            printf("Warning - %s [channel %02d] - Sample voltage is out of the thresholds between 0x%04x and 0x%04x !\n", BOARD_APP_ADC16_NAME, i, threshold.thshdl, threshold.thshdh);
    //        }
    //    }
    //
    //    res_out_of_thr_flag = 0;
    //    adc16_enable_interrupts(HPM_ADC0, APP_ADC16_CH_WDOG_EVENT);
    //}
}

void init_sequence_config(void)
{
    {
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = APP_ADC16_CH_SAMPLE_CYCLE;

    for (uint32_t i = 0; i < sizeof(seq_adc_channel0); i++) {
        ch_cfg.ch           = seq_adc_channel0[i];
        adc16_init_channel(HPM_ADC0, &ch_cfg);
    }

    /* Set a sequence config */
    seq_cfg.seq_len    = sizeof(seq_adc_channel0);
    seq_cfg.restart_en = true;
    seq_cfg.cont_en    = true;
#ifndef ADC_SOC_NO_HW_TRIG_SRC
    #ifndef __ADC16_USE_SW_TRIG
    seq_cfg.hw_trig_en = true;
    seq_cfg.sw_trig_en = false;
    #else
    seq_cfg.hw_trig_en = false;
    seq_cfg.sw_trig_en = true;
    #endif
#else
    seq_cfg.hw_trig_en = false;
    seq_cfg.sw_trig_en = true;
#endif

    for (int i = APP_ADC16_SEQ_START_POS; i < seq_cfg.seq_len; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = seq_adc_channel0[i];
    }

    /* Enable the single complete interrupt for the last conversion */
    seq_cfg.queue[seq_cfg.seq_len - 1].seq_int_en = true;

    /* Initialize a sequence */
    adc16_set_seq_config(HPM_ADC0, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc0_seq_buff);
    dma_cfg.buff_len_in_4bytes = sizeof(seq_adc_channel0)*APP_SAMPLE_LENGTH;
    dma_cfg.stop_en            = false;
    dma_cfg.stop_pos           = 0;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(HPM_ADC0, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_full_complete);

#if !defined(ADC_SOC_NO_HW_TRIG_SRC) && !defined(__ADC16_USE_SW_TRIG)
    /* Trigger mux initialization */
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC0_STRGI);

    /* Trigger source initialization */
    init_trigger_source(APP_ADC16_HW_TRIG_SRC);
#endif
    }
    {
        
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = APP_ADC16_CH_SAMPLE_CYCLE;

    for (uint32_t i = 0; i < sizeof(seq_adc_channel1); i++) {
        ch_cfg.ch           = seq_adc_channel1[i];
        adc16_init_channel(HPM_ADC1, &ch_cfg);
    }

    /* Set a sequence config */
    seq_cfg.seq_len    = sizeof(seq_adc_channel1);
    seq_cfg.restart_en = true;
    seq_cfg.cont_en    = true;
#ifndef ADC_SOC_NO_HW_TRIG_SRC
    #ifndef __ADC16_USE_SW_TRIG
    seq_cfg.hw_trig_en = true;
    seq_cfg.sw_trig_en = false;
    #else
    seq_cfg.hw_trig_en = false;
    seq_cfg.sw_trig_en = true;
    #endif
#else
    seq_cfg.hw_trig_en = false;
    seq_cfg.sw_trig_en = true;
#endif

    for (int i = APP_ADC16_SEQ_START_POS; i < seq_cfg.seq_len; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = seq_adc_channel1[i];
    }

    /* Enable the single complete interrupt for the last conversion */
    seq_cfg.queue[seq_cfg.seq_len - 1].seq_int_en = true;

    /* Initialize a sequence */
    adc16_set_seq_config(HPM_ADC1, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc1_seq_buff);
    dma_cfg.buff_len_in_4bytes = sizeof(seq_adc_channel1)*APP_SAMPLE_LENGTH;
    dma_cfg.stop_en            = false;
    dma_cfg.stop_pos           = 0;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(HPM_ADC1, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(HPM_ADC1, adc16_event_seq_full_complete);

#if !defined(ADC_SOC_NO_HW_TRIG_SRC) && !defined(__ADC16_USE_SW_TRIG)
    /* Trigger mux initialization */
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC1_STRGI);

    /* Trigger source initialization */
    init_trigger_source(APP_ADC16_HW_TRIG_SRC);
#endif
    }
}

bool abort_handler(uint8_t conv_mode)
{
    if (console_try_receive_byte() == ' ') {

    #if !defined(ADC_SOC_NO_HW_TRIG_SRC) && !defined(__ADC16_USE_SW_TRIG)
        if (conv_mode == adc16_conv_mode_sequence) {
            adc16_seq_disable_hw_trigger(HPM_ADC0);
        }

        stop_trigger_source(APP_ADC16_HW_TRIG_SRC);
    #else
        (void) conv_mode;
    #endif
        current_cycle_bit = 0;

        return true;
    } else {
        return false;
    }
}

bool abort_handler1(uint8_t conv_mode)
{
    if (console_try_receive_byte() == ' ') {

    #if !defined(ADC_SOC_NO_HW_TRIG_SRC) && !defined(__ADC16_USE_SW_TRIG)
        if (conv_mode == adc16_conv_mode_sequence) {
            adc16_seq_disable_hw_trigger(HPM_ADC1);
        }

        stop_trigger_source(APP_ADC16_HW_TRIG_SRC);
    #else
        (void) conv_mode;
    #endif
        current_cycle_bit = 0;

        return true;
    } else {
        return false;
    }
}

int adc_init(void)
{
    //printf("This is an ADC16 demo:\n");
    /* ADC16 common initialization */
    init_common_config();
    init_sequence_config();

    adc16_trigger_seq_by_sw(HPM_ADC0);
    adc16_trigger_seq_by_sw(HPM_ADC1);
    //while (1) {
    //    sequence_handler1();
    //    if (abort_handler(conv_mode)) {
    //        break;
    //    }
    //}
    return 0;
}


SDK_DECLARE_EXT_ISR_M(IRQn_ADC0, isr_adc0)
void isr_adc0(void)
{
    uint32_t status;

    status = adc16_get_status_flags(HPM_ADC0);

    /* Clear status */
    adc16_clear_status_flags(HPM_ADC0, status);

    if (ADC16_INT_STS_SEQ_CVC_GET(status)) {
        /* Set flag to read memory data */
        seq_complete_flag = 1;
    }
    printf("adc0\n");
    //if (ADC16_INT_STS_WDOG_GET(status) & APP_ADC16_CH_WDOG_EVENT) {
    //    adc16_disable_interrupts(HPM_ADC0, APP_ADC16_CH_WDOG_EVENT);
    //    res_out_of_thr_flag = ADC16_INT_STS_WDOG_GET(status) & APP_ADC16_CH_WDOG_EVENT;
    //}
}

SDK_DECLARE_EXT_ISR_M(IRQn_ADC1, isr_adc1)
void isr_adc1(void)
{
    uint32_t status;

    status = adc16_get_status_flags(HPM_ADC1);

    /* Clear status */
    adc16_clear_status_flags(HPM_ADC1, status);

    if (ADC16_INT_STS_SEQ_CVC_GET(status)) {
        /* Set flag to read memory data */
        seq_complete_flag = 1;
    }

    //if (ADC16_INT_STS_WDOG_GET(status) & APP_ADC16_CH_WDOG_EVENT) {
    //    adc16_disable_interrupts(HPM_ADC1, APP_ADC16_CH_WDOG_EVENT);
    //    res_out_of_thr_flag = ADC16_INT_STS_WDOG_GET(status) & APP_ADC16_CH_WDOG_EVENT;
    //}
}
#include "gptmr.h"
/* 头部丢弃：避开 MUX 切换后的震荡 (固定丢弃 Buffer 开头的数据) */
#define ADC_MUX_DISCARD_COUNT_BEGIN     (5U) 
/* 真正的通道数量 */
#define REAL_CHANNEL_COUNT              (5U)
void update_ringbuf()
{
    // 1. 读取当前 DMA 到底写了多少数据 (这就是上一轮 MUX 通道的有效数据量)
    // 注意：因为我们每次都复位了 DMA，所以 SEQ_WR_ADDR 直接就是有效长度，不需要减去 last_ptr
    uint32_t valid_len0 = ADC16_SEQ_WR_ADDR_SEQ_WR_POINTER_GET(HPM_ADC0->SEQ_WR_ADDR);
    uint32_t valid_len1 = ADC16_SEQ_WR_ADDR_SEQ_WR_POINTER_GET(HPM_ADC1->SEQ_WR_ADDR);

    // 2. 映射缓冲区
    adc16_seq_dma_data_t *dma_data0 = (adc16_seq_dma_data_t *)adc0_seq_buff;
    adc16_seq_dma_data_t *dma_data1 = (adc16_seq_dma_data_t *)adc1_seq_buff;

    // 3. 处理上一轮的数据 (此时 g_analog_active_channel 指向的就是这批数据的归属)
    // =========================================================================
    uint32_t sum_values[10] = {0};
    uint32_t sample_counts[10] = {0};
    
    // --- 处理 ADC0 ---
    if (valid_len0 > ADC_MUX_DISCARD_COUNT_BEGIN)
    {
        // 直接从 [DISCARD] 开始读，读到 [valid_len0]
        // 这里的 Buffer[0] 就是 MUX 切换瞬间采的，Buffer[50] 以后就是稳定的
        for (uint32_t i = ADC_MUX_DISCARD_COUNT_BEGIN; i < valid_len0; i++)
        {
            adc16_seq_dma_data_t* val = &dma_data0[i];
            
            uint32_t logic_ch_idx = val->seq_num;
            sum_values[0 + logic_ch_idx] += val->result;
            sample_counts[0 + logic_ch_idx]++;
        }
    }

    // --- 处理 ADC1 ---
    if (valid_len1 > ADC_MUX_DISCARD_COUNT_BEGIN)
    {
        for (uint32_t i = ADC_MUX_DISCARD_COUNT_BEGIN; i < valid_len1; i++)
        {
            adc16_seq_dma_data_t* val = &dma_data1[i];

            uint32_t logic_ch_idx = val->seq_num;
            sum_values[5 + logic_ch_idx] += val->result;
            sample_counts[5 + logic_ch_idx]++;
        }
    }
    extern uint32_t debug;
    debug += sample_counts[1];
    // --- 推入 RingBuf ---
    for (size_t j = 0; j < REAL_CHANNEL_COUNT; j++)
    {
        if (sample_counts[0 + j] > 0) 
            ringbuf_push(&g_adc_ringbufs[0 + j * 8 + g_analog_active_channel], sum_values[0 + j] / sample_counts[0 + j]);
        if (sample_counts[5 + j] > 0)
            ringbuf_push(&g_adc_ringbufs[40 + j * 8 + g_analog_active_channel], sum_values[5 + j] / sample_counts[5 + j]);
    }

    // 4. 准备下一轮采样 (关键步骤：复位 -> 切换 -> 重启)
    // =========================================================================
    
    // A. 预计算下一个通道
    uint32_t next_channel = g_analog_active_channel + 1;
    if (next_channel >= 7) next_channel = 0;

    // B. 复位 DMA 指针 (这会让写指针立即回到 Buffer[0])
    // 注意：设置 DMA_RST 位会挂起 DMA
    HPM_ADC0->SEQ_DMA_CFG |= ADC16_SEQ_DMA_CFG_DMA_RST_MASK;
    HPM_ADC1->SEQ_DMA_CFG |= ADC16_SEQ_DMA_CFG_DMA_RST_MASK;

    // C. 切换 MUX (此时 DMA 是暂停的，绝对安全)
    analog_channel_select(next_channel);
    g_analog_active_channel = next_channel;

    // D. 释放 DMA (ADC 将立即开始往 Buffer[0] 写入新通道的数据)
    // 写入这一行的瞬间，就是下一轮数据的 Time = 0
    HPM_ADC0->SEQ_DMA_CFG &= ~ADC16_SEQ_DMA_CFG_DMA_RST_MASK;
    HPM_ADC1->SEQ_DMA_CFG &= ~ADC16_SEQ_DMA_CFG_DMA_RST_MASK;

    gptmr_channel_reset_count(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
    gptmr_start_counter(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
}
