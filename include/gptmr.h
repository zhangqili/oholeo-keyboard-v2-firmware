/*
 * Copyright (c) 2024 Zhangqi Li (@zhangqili)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef GPTMR_H_
#define GPTMR_H_


#include "hpm_gptmr_drv.h"

#define KEYBOARD_TICK_GPTMR               HPM_GPTMR0
#define KEYBOARD_TICK_GPTMR_CH            0
#define KEYBOARD_TICK_GPTMR_IRQ           IRQn_GPTMR0

#define RINGBUF_TICK_GPTMR               HPM_GPTMR1
#define RINGBUF_TICK_GPTMR_CH            0
#define RINGBUF_TICK_GPTMR_IRQ           IRQn_GPTMR1

int gptmr_init(void);
void keyboard_tick_timer_config(void);

#endif /* GPTMR_H_ */
