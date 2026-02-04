/*
 * Copyright (c) 2025 Zhangqi Li (@zhangqili)
 *
 * SPDX-License-Identifier: MIT
 */
#include "ws2812.h"
#include "board.h"
#include "hpm_spi_drv.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_l1c_drv.h"

    //if (index >= WS2812_LED_NUM) return;
#if WS2812_INTERFACE == WS2812_USE_PWM
#define WS2812_BUFFER_LENGTH    (((WS2812_LED_NUM)*(3*8))+WS2812_RESET_LENGTH)
#elif WS2812_INTERFACE == WS2812_USE_SPI
#define WS2812_BUFFER_LENGTH    ((WS2812_LED_NUM * 3 * 4)+WS2812_RESET_LENGTH)
#endif


#define WS2812_SPI              HPM_SPI3
#define WS2812_SPI_DMA          BOARD_APP_HDMA
#define WS2812_SPI_DMAMUX       BOARD_APP_DMAMUX
#define WS2812_SPI_TX_DMA_REQ   HPM_DMA_SRC_SPI3_TX
#define WS2812_SPI_TX_DMA_CH    1
#define WS2812_SPI_TX_DMAMUX_CH DMA_SOC_CHN_TO_DMAMUX_CHN(WS2812_SPI_DMA, WS2812_SPI_TX_DMA_CH)

#define SPI_DMA_TRANS_DATA_WIDTH DMA_TRANSFER_WIDTH_BYTE

static uint8_t ws2812_buffer[WS2812_BUFFER_LENGTH];
static spi_control_config_t control_config = {0};

hpm_stat_t spi_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t src, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_t config;
    
    /* 确保 SPI TX DMA 在配置前关闭 */
    spi_disable_tx_dma(spi_ptr);
    
    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = (uint32_t)&spi_ptr->DATA;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = false;
    config.data_width = data_width;
    config.size_in_byte = size;
    
    config.en_infiniteloop = true;

    return dma_setup_handshake(dma_ptr, &config, true);
}

void ws2812_spi_init(void)
{
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    uint32_t spi_clock_freq;

    // 1. 初始化引脚和时钟
    spi_clock_freq = board_init_spi_clock(WS2812_SPI);
    board_init_spi_pins(WS2812_SPI); 
    
    printf("WS2812 Init...\n");

    // 2. SPI 时序 (3.2MHz)
    //spi_master_get_default_timing_config(&timing_config);
    //timing_config.master_config.clk_src_freq_in_hz = spi_clock_freq;
    //timing_config.master_config.sclk_freq_in_hz = 32000000;
    uint32_t target_div = 24; // 80MHz / 24 = 3.33MHz
    uint32_t sclk_div_reg = (target_div / 2) - 1;
    WS2812_SPI->TIMING = SPI_TIMING_CS2SCLK_SET(spi_cs2sclk_half_sclk_4) |
                  SPI_TIMING_CSHT_SET(spi_csht_half_sclk_12) |
                  SPI_TIMING_SCLK_DIV_SET(sclk_div_reg);
    //if (status_success != spi_master_timing_init(WS2812_SPI, &timing_config)) {
    //    printf("SPI timing init failed\n");
    //    while (1);
    //}

    // 3. SPI 格式
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 8;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle; 
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    spi_format_init(WS2812_SPI, &format_config);

    // 4. SPI 控制 (Write Only, No CMD/ADDR)
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;
    control_config.master_config.addr_enable = false;
    control_config.common_config.tx_dma_enable = true;
    control_config.common_config.rx_dma_enable = false;
    control_config.common_config.trans_mode = spi_trans_write_only;
    control_config.common_config.data_phase_fmt = spi_single_io_mode;
    spi_control_init(WS2812_SPI, &control_config, sizeof(ws2812_buffer), 0);


    dmamux_config(WS2812_SPI_DMAMUX, WS2812_SPI_TX_DMAMUX_CH, WS2812_SPI_TX_DMA_REQ, true);
}

void ws2812_init(void)
{
    ws2812_spi_init();
#if WS2812_INTERFACE == WS2812_USE_PWM
    for (uint16_t i = 0; i < WS2812_BUFFER_LENGTH; i++)
    {
        ws2812_buffer[i] = NONE_PULSE;
    }
#elif WS2812_INTERFACE == WS2812_USE_SPI
    memset(ws2812_buffer, 0, sizeof(ws2812_buffer));
    for (uint16_t i = 0; i < WS2812_BUFFER_LENGTH; i++)
    {
        ws2812_set(i, 0, 0, 0);
    };
#endif
}

void ws2812_demo(void)
{
    hpm_stat_t stat;
    uint32_t step = 0;

    while (1) {
        // 1. 清除颜色数据 (复位区保持 0)
        memset(ws2812_buffer, 0, sizeof(ws2812_buffer)); 
        static uint8_t counter = 0;
        counter++;
        // 2. 设置跑马灯
        uint32_t active_led = step % WS2812_LED_NUM;
        for (int i = 0; i < WS2812_LED_NUM; i++)
        {
            ws2812_set(i, 0, 0, 0); 
        }
        
        ws2812_set(active_led, 0xFF, 0xFF, 0xFF); 
        step++;

        // 3. 配置并启动 DMA
        
        ws2812_flush();

        // 4. 等待完成
        //while (spi_is_active(WS2812_SPI));
        
        board_delay_ms(50);
    }
}

int ws2812_flush(void)
{
    if (spi_is_active(WS2812_SPI))
    {
        return 1;
    }
    hpm_stat_t stat;
    stat = spi_tx_trigger_dma(WS2812_SPI_DMA,
                            WS2812_SPI_TX_DMA_CH,
                            WS2812_SPI,
                            core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)ws2812_buffer),
                            SPI_DMA_TRANS_DATA_WIDTH,
                            sizeof(ws2812_buffer));
    
    if (stat == status_success) {
        stat = spi_setup_dma_transfer(WS2812_SPI,
                            &control_config,
                            NULL, NULL,
                            sizeof(ws2812_buffer), 0);
    }
    return 0;
}


void ws2812_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_NUM) return;
#if WS2812_INTERFACE == WS2812_USE_PWM
    for (uint8_t i = 0; i < 8; i++)
    {
        ws2812_buffer[WS2812_RESET_LENGTH + index * 24 + i] = (g << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        ws2812_buffer[WS2812_RESET_LENGTH + index * 24 + i + 8] = (r << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        ws2812_buffer[WS2812_RESET_LENGTH + index * 24 + i + 16] = (b << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
    }
#elif WS2812_INTERFACE == WS2812_USE_SPI
    uint8_t *p = &ws2812_buffer[index * 12];
    uint8_t color_bytes[3] = {g, r, b};
    for (int i = 0; i < 3; i++) {
        uint8_t val = color_bytes[i];
        for (int bit = 7; bit >= 0; bit -= 2) {
            uint8_t spi_byte = 0;
            if (val & (1 << bit)) spi_byte |= (WS2812_CODE_1 << 4);
            else                  spi_byte |= (WS2812_CODE_0 << 4);
            if (val & (1 << (bit - 1))) spi_byte |= WS2812_CODE_1;
            else                        spi_byte |= WS2812_CODE_0;
            *p++ = spi_byte;
        }
    }
#endif
}
