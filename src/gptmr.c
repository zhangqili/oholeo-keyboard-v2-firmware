/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "gptmr.h"
#include <stdio.h>
#include "board.h"
#include "hpm_sysctl_drv.h"
#include "hpm_debug_console.h"

void keyboard_tick_timer_config(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_stop_counter(KEYBOARD_TICK_GPTMR,KEYBOARD_TICK_GPTMR_CH);
    gptmr_channel_reset_count(KEYBOARD_TICK_GPTMR,KEYBOARD_TICK_GPTMR_CH);
    gptmr_freq = board_init_gptmr_clock(KEYBOARD_TICK_GPTMR);
    //keyboard tick
    gptmr_channel_get_default_config(KEYBOARD_TICK_GPTMR, &config);
    printf("ring_buf_gptmr_freq%ld\n", gptmr_freq);

    config.reload = 0xFFFFFFFF;
    config.cmp[0] = 7500;
    //config.enable_opmode = true;
    gptmr_channel_config(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH, &config, true);
    //gptmr_channel_enable_opmode(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
    gptmr_enable_irq(KEYBOARD_TICK_GPTMR, GPTMR_CH_CMP_IRQ_MASK(KEYBOARD_TICK_GPTMR_CH, 0));
    intc_m_enable_irq_with_priority(KEYBOARD_TICK_GPTMR_IRQ, 2);
}

static void keyboard_tick_timer_config1(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_freq = board_init_gptmr_clock(KEYBOARD_TICK_GPTMR);
    //keyboard tick
    gptmr_channel_get_default_config(KEYBOARD_TICK_GPTMR, &config);
    printf("ring_buf_gptmr_freq%ld\n", gptmr_freq);

    config.reload = gptmr_freq / 8000 - 1;
    config.cmp[0] = gptmr_freq / 8000/2 - 1;
    //config.enable_opmode = true;
    gptmr_channel_config(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH, &config, false);
    //gptmr_channel_enable_opmode(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
    gptmr_enable_irq(KEYBOARD_TICK_GPTMR, GPTMR_CH_CMP_IRQ_MASK(KEYBOARD_TICK_GPTMR_CH, 0));
    intc_m_enable_irq_with_priority(KEYBOARD_TICK_GPTMR_IRQ, 2);
}

static void ring_buf_timer_config(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_freq = board_init_gptmr_clock(RINGBUF_TICK_GPTMR);
    //ring buf
    gptmr_channel_get_default_config(RINGBUF_TICK_GPTMR, &config);
    printf("ring_buf_gptmr_freq%ld\n", gptmr_freq);

    config.reload = gptmr_freq / 40000 - 1;
    config.enable_opmode = true;
    gptmr_channel_config(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH, &config, false);
    gptmr_channel_enable_opmode(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
    gptmr_enable_irq(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_IRQ_MASK(RINGBUF_TICK_GPTMR_CH));
    intc_m_enable_irq_with_priority(RINGBUF_TICK_GPTMR_IRQ, 1);
}


int gptmr_init(void)
{
    printf("gptmr timer basic test\n");
    keyboard_tick_timer_config1();
    ring_buf_timer_config();
    return 0;
}


SDK_DECLARE_EXT_ISR_M(KEYBOARD_TICK_GPTMR_IRQ, keyboard_tick_isr)
void keyboard_tick_isr(void)
{
    if (gptmr_check_status(KEYBOARD_TICK_GPTMR, GPTMR_CH_CMP_STAT_MASK(KEYBOARD_TICK_GPTMR_CH, 0))) {
        gptmr_clear_status(KEYBOARD_TICK_GPTMR, GPTMR_CH_CMP_STAT_MASK(KEYBOARD_TICK_GPTMR_CH, 0));
        void keyboard_tick_task(void);
        keyboard_tick_task();
    }
}

SDK_DECLARE_EXT_ISR_M(RINGBUF_TICK_GPTMR_IRQ, ring_buf_isr)
void ring_buf_isr(void)
{
    if (gptmr_check_status(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(RINGBUF_TICK_GPTMR_CH))) {
        gptmr_clear_status(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(RINGBUF_TICK_GPTMR_CH));
        void update_ringbuf(void);
        update_ringbuf();
    }
}
