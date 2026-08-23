/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "board_api.h"
#include "hpm_romapi.h"
#include "hpm_ppor_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_adc16_drv.h"
#include "hpm_otp_drv.h"
#include "hpm_soc_feature.h"
#include "ws2812.h"

#define BOOT_KEY_MUX_CHANNEL            6U
#define BOOT_KEY_ADC                    HPM_ADC1
#define BOOT_KEY_ADC_CHANNEL            15U
#define BOOT_KEY_ADC_SAMPLE_CYCLE       15U
#define BOOT_KEY_MUX_SETTLE_US          100U
#define BOOT_KEY_DISCARD_SAMPLE_COUNT   3U
#define BOOT_KEY_SAMPLE_COUNT           8U
#define BOOT_KEY_LOW_THRESHOLD          8192U
#define BOOT_KEY_HIGH_THRESHOLD         (65536U - BOOT_KEY_LOW_THRESHOLD)


void uf2_board_init(void)
{
    board_init();
    board_init_gpio_pins();
}

void uf2_board_app_jump(void)
{
    fencei();
    l1c_dc_disable();
    __asm("la a0, %0" ::"i"(BOARD_FLASH_APP_START + 4));
    __asm("jr a0");
}

bool uf2_board_app_valid(void)
{
    if (((volatile uint32_t const*) BOARD_FLASH_APP_START)[0] != BOARD_UF2_SIGNATURE) {
        return false;
    }

    return true;
}

void uf2_board_dfu_complete(void)
{
    ppor_sw_reset(HPM_PPOR, 10);
}

void uf2_board_dfu_init(void)
{
    ws2812_init();
}

static xpi_nor_config_t s_xpi_nor_config;

void uf2_board_flash_init(void)
{
    xpi_nor_config_option_t option;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
    rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &s_xpi_nor_config, &option);
}

#define NO_CACHE        0xffffffff
#define SECTOR_SIZE     (4*1024)
static uint32_t _flash_page_addr = NO_CACHE;
static uint8_t  _flash_cache[SECTOR_SIZE] __attribute__((aligned(HPM_L1C_CACHELINE_SIZE)));

void uf2_board_flash_flush(void)
{

    hpm_stat_t status;

    if (_flash_page_addr == NO_CACHE) {
        return;
    }

    printf("Erase and Write at address = 0x%08lX\r\n", _flash_page_addr);

    /* Skip if data is the same */
    if (memcmp(_flash_cache, (void *) _flash_page_addr, SECTOR_SIZE) != 0) {
        uint32_t const sector_addr = (_flash_page_addr - BOARD_FLASH_BASE_ADDRESS);

        disable_global_irq(CSR_MSTATUS_MIE_MASK);
        status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config,
                sector_addr, SECTOR_SIZE);
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
        l1c_dc_invalidate(_flash_page_addr, SECTOR_SIZE);
        if (status != status_success) {
            printf("Erase failed: status = %ld!\r\n", status);
            return;
        }

        l1c_dc_writeback(_flash_page_addr, SECTOR_SIZE);
        disable_global_irq(CSR_MSTATUS_MIE_MASK);
        status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config,
                (uint32_t *)_flash_cache, sector_addr, SECTOR_SIZE);
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
        if (status != status_success) {
            printf("Page program failed: status = %ld!\r\n", status);
            return;
        }


        l1c_dc_invalidate(_flash_page_addr, SECTOR_SIZE);
    }

    _flash_page_addr = NO_CACHE;
}

void uf2_board_flash_abort(void)
{
    /* The cache has not been programmed until uf2_board_flash_flush(). */
    _flash_page_addr = NO_CACHE;
}

void uf2_board_flash_invalidate_app(void)
{
    const uint32_t invalid_signature = 0;

    /*
     * BOARD_UF2_SIGNATURE is also the boot-valid marker. Program an invalid
     * value and flush it before touching the image, so a reset during DFU
     * always returns to the bootloader instead of jumping into a partial app.
     */
    uf2_board_flash_write(BOARD_FLASH_APP_START, &invalid_signature, sizeof(invalid_signature));
    uf2_board_flash_flush();
}


void uf2_board_flash_read(uint32_t addr, void *buffer, uint32_t len)
{
    uint32_t const flash_offset = addr - BOARD_FLASH_BASE_ADDRESS;

    /* ROM XPI APIs use a byte offset in the NOR device, not its XIP address. */
    if ((addr < BOARD_FLASH_BASE_ADDRESS)
        || (flash_offset > BOARD_FLASH_SIZE)
        || (len > (BOARD_FLASH_SIZE - flash_offset))
        || (rom_xpi_nor_read(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto,
                             &s_xpi_nor_config, buffer, flash_offset, len) != status_success)) {
        /* This API cannot return an error to its callers; never expose stale RAM. */
        memset(buffer, 0xff, len);
    }
}

uint32_t uf2_board_flash_size(void)
{
    return ((UF2_AVAILABLE_BOARD_FLASH_SIZE) > 8 << 20) ? 8 << 20 : UF2_AVAILABLE_BOARD_FLASH_SIZE;
}

void uf2_board_flash_write(uint32_t addr, void const *src, uint32_t len)
{
    uint32_t const page_addr = addr & ~(SECTOR_SIZE - 1);

    if (page_addr != _flash_page_addr) {
        /* Write out anything in cache before overwriting it. */
        uf2_board_flash_flush();

        _flash_page_addr = page_addr;

        /* Copy the current contents of the entire page into the cache. */
        memcpy(_flash_cache, (void *) page_addr, SECTOR_SIZE);
    }

    /* Overwrite part or all of the page cache with the src data. */
    memcpy(_flash_cache + (addr & (SECTOR_SIZE - 1)), src, len);
}

void uf2_board_pwm_rgb_write(uint8_t *rgb)
{
    if (rgb == NULL) {
        return;
    }

    for (uint16_t index = 0; index < WS2812_LED_NUM; index++) {
        ws2812_set(index, rgb[0], rgb[1], rgb[2]);
    }

    /* State transitions are infrequent. If the previous frame is still being
     * shifted out, wait for it and then submit the complete new frame. */
    while (ws2812_flush() != 0) {
    }
}

void uf2_board_pwm_led_write(uint8_t value)
{
}

void uf2_board_timer_start(uint32_t d)
{
}

void uf2_board_timer_stop(void)
{
}

#define HPM_UUID_WORD_SIZE  4U
#define HPM_UUID_WORD_COUNT (OTP_SOC_UUID_LEN / HPM_UUID_WORD_SIZE)

#if (OTP_SOC_UUID_LEN % HPM_UUID_WORD_SIZE) != 0
#error OTP_SOC_UUID_LEN must be word aligned
#endif

uint8_t uf2_board_usb_get_serial(uint8_t *id)
{
    size_t id_index = 0;

    if (id == NULL) {
        return 0;
    }

    /* Match the main application serial exactly: read the OTP UUID words and
     * expand each word little-endian before usb_descriptors.c converts it to
     * its hexadecimal USB string representation. */
    for (size_t word_index = 0; word_index < HPM_UUID_WORD_COUNT; word_index++) {
        const uint32_t word = otp_read_from_shadow(OTP_SOC_UUID_IDX + word_index);

        for (size_t byte_index = 0; byte_index < HPM_UUID_WORD_SIZE; byte_index++) {
            if (id_index >= 16U) {
                return (uint8_t)id_index;
            }

            id[id_index++] = (uint8_t)((word >> (byte_index * 8U)) & 0xFFU);
        }
    }

    return (uint8_t)id_index;
}

bool uf2_board_enter_bootloader(void)
{
    uint32_t counter = 0;
    gpio_set_pin_input(UF2_BOOTLOADER_PIN_GPIO_CTRL, UF2_BOOTLOADER_PIN_GPIO_INDEX, UF2_BOOTLOADER_PIN_GPIO_PIN);
    for (uint8_t i = 0; i < UF2_BOOTLOADER_SAMPLE_COUNT; i++) {
        if (UF2_BOOTLOADER_PIN_ACTIVE == gpio_read_pin(UF2_BOOTLOADER_PIN_GPIO_CTRL, UF2_BOOTLOADER_PIN_GPIO_INDEX, UF2_BOOTLOADER_PIN_GPIO_PIN)) {
            counter++;
        }
        board_delay_ms(10);
    }
    if (counter > 0.6 * UF2_BOOTLOADER_SAMPLE_COUNT) {
        return true;
    }
    return false;
}

bool uf2_board_boot_key_requests_dfu(void)
{
    adc16_config_t adc_config;
    adc16_channel_config_t channel_config;
    uint32_t sample_sum = 0;
    uint32_t sample_average;
    uint16_t sample;

    /* Key 0 maps to g_analog_map index 78 in the application, which is
     * ADC1 sequence slot 4 (physical channel 15) on mux channel 6. */
    for (uint8_t pin = 0; pin < 3U; pin++) {
        gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOY, pin, gpiom_core0_fast);
        gpio_set_pin_output(HPM_FGPIO, GPIO_OE_GPIOY, pin);
    }
    gpio_write_pin(HPM_FGPIO, GPIO_DO_GPIOY, 0, BOOT_KEY_MUX_CHANNEL & 0x01U);
    gpio_write_pin(HPM_FGPIO, GPIO_DO_GPIOY, 1, BOOT_KEY_MUX_CHANNEL & 0x02U);
    gpio_write_pin(HPM_FGPIO, GPIO_DO_GPIOY, 2, BOOT_KEY_MUX_CHANNEL & 0x04U);

    board_init_adc16_pins();
    board_init_adc_clock(BOOT_KEY_ADC, true);

    adc16_get_default_config(&adc_config);
    adc_config.res = adc16_res_16_bits;
    adc_config.conv_mode = adc16_conv_mode_oneshot;
    adc_config.adc_clk_div = adc16_clock_divider_4;
    adc_config.sel_sync_ahb = true;

    if (adc16_init(BOOT_KEY_ADC, &adc_config) != status_success) {
        printf("Boot key ADC initialization failed; staying in DFU mode\r\n");
        return true;
    }

    adc16_get_channel_default_config(&channel_config);
    channel_config.ch = BOOT_KEY_ADC_CHANNEL;
    channel_config.sample_cycle = BOOT_KEY_ADC_SAMPLE_CYCLE;
    if (adc16_init_channel(BOOT_KEY_ADC, &channel_config) != status_success) {
        printf("Boot key ADC channel initialization failed; staying in DFU mode\r\n");
        adc16_deinit(BOOT_KEY_ADC);
        return true;
    }

#if defined(ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT) && ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT
    adc16_enable_oneshot_mode(BOOT_KEY_ADC);
#endif
    adc16_set_blocking_read(BOOT_KEY_ADC);
    board_delay_us(BOOT_KEY_MUX_SETTLE_US);

    for (uint32_t index = 0;
         index < (BOOT_KEY_DISCARD_SAMPLE_COUNT + BOOT_KEY_SAMPLE_COUNT);
         index++) {
        if (adc16_get_oneshot_result(BOOT_KEY_ADC, BOOT_KEY_ADC_CHANNEL, &sample)
            != status_success) {
            printf("Boot key ADC sampling failed; staying in DFU mode\r\n");
#if defined(ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT) && ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT
            adc16_disable_oneshot_mode(BOOT_KEY_ADC);
#endif
            adc16_deinit(BOOT_KEY_ADC);
            return true;
        }

        if (index >= BOOT_KEY_DISCARD_SAMPLE_COUNT) {
            sample_sum += sample;
        }
    }

#if defined(ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT) && ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT
    adc16_disable_oneshot_mode(BOOT_KEY_ADC);
#endif
    adc16_deinit(BOOT_KEY_ADC);

    sample_average = sample_sum / BOOT_KEY_SAMPLE_COUNT;
    printf("Boot key ADC average: %lu\r\n", (unsigned long) sample_average);

    return (sample_average < BOOT_KEY_LOW_THRESHOLD)
        || (sample_average > BOOT_KEY_HIGH_THRESHOLD);
}
