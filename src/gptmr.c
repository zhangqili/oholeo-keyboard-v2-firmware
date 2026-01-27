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
#include "keyboard.h"
#include "analog.h"
#include "hpm_adc16_drv.h"

void update_ringbuf()
{
  extern uint32_t seq_buff0[1024];
  extern uint32_t seq_buff1[1024];
  adc16_seq_dma_data_t *dma_data0 = (adc16_seq_dma_data_t *)seq_buff0;
  adc16_seq_dma_data_t *dma_data1 = (adc16_seq_dma_data_t *)seq_buff1;
  uint32_t adc_values[10] = {0};

  for (int i = 0; i < 2; i++)
  {
    for (size_t j = 0; j < 5; j++)
    {
        adc_values[0+j] += dma_data0[i*5+j].result;
        adc_values[5+j] += dma_data1[i*5+j].result;
    }
  }
  for (size_t j = 0; j < 5; j++)
  {
    ringbuf_push(&g_adc_ringbufs[0  + j * 8 + (g_analog_active_channel)], adc_values[0+j]>>2);
    ringbuf_push(&g_adc_ringbufs[40 + j * 8 + (g_analog_active_channel)], adc_values[5+j]>>2);
  }
  g_analog_active_channel++;
  if (g_analog_active_channel >= 7)
  {
    g_analog_active_channel = 0;
  }
  analog_channel_select(g_analog_active_channel);
}

static void keyboard_tick_timer_config(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_freq = board_init_gptmr_clock(KEYBOARD_TICK_GPTMR);
    //keyboard tick
    gptmr_channel_get_default_config(KEYBOARD_TICK_GPTMR, &config);

    config.reload = gptmr_freq / 8000 * 1;
    gptmr_channel_config(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH, &config, false);

    gptmr_enable_irq(KEYBOARD_TICK_GPTMR, GPTMR_CH_RLD_IRQ_MASK(KEYBOARD_TICK_GPTMR_CH));
    intc_m_enable_irq_with_priority(KEYBOARD_TICK_GPTMR_IRQ, 1);
}

static void ring_buf_timer_config(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_freq = board_init_gptmr_clock(RINGBUF_TICK_GPTMR);
    //ring buf
    gptmr_channel_get_default_config(RINGBUF_TICK_GPTMR, &config);

    config.reload = gptmr_freq / 3000 * 1;
    gptmr_channel_config(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH, &config, false);

    gptmr_enable_irq(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_IRQ_MASK(RINGBUF_TICK_GPTMR_CH));
    intc_m_enable_irq_with_priority(RINGBUF_TICK_GPTMR_IRQ, 2);
}


int gptmr_init(void)
{
    printf("gptmr timer basic test\n");
    keyboard_tick_timer_config();
    ring_buf_timer_config();
    return 0;
}


SDK_DECLARE_EXT_ISR_M(KEYBOARD_TICK_GPTMR_IRQ, keyboard_tick_isr)
void keyboard_tick_isr(void)
{
    if (gptmr_check_status(KEYBOARD_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(KEYBOARD_TICK_GPTMR_CH))) {
        gptmr_clear_status(KEYBOARD_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(KEYBOARD_TICK_GPTMR_CH));
        g_keyboard_tick++;
        //keyboard_task();
    }
}

SDK_DECLARE_EXT_ISR_M(RINGBUF_TICK_GPTMR_IRQ, ring_buf_isr)
void ring_buf_isr(void)
{
    if (gptmr_check_status(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(RINGBUF_TICK_GPTMR_CH))) {
        gptmr_clear_status(RINGBUF_TICK_GPTMR, GPTMR_CH_RLD_STAT_MASK(RINGBUF_TICK_GPTMR_CH));
        update_ringbuf();
    }
}
