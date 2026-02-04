/*
 * Copyright (c) 2025 Zhangqi Li (@zhangqili)
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef WS2812_H_
#define WS2812_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WS2812_USE_PWM          0
#define WS2812_USE_SPI          1
#define WS2812_INTERFACE        WS2812_USE_SPI
#define WS2812_LED_NUM          (69)
#define ONE_PULSE               (60)
#define ZERO_PULSE              (29)
#define NONE_PULSE              (0)
#define WS2812_RESET_LENGTH     (400)
#define WS2812_CODE_0           0x8
#define WS2812_CODE_1           0xE

void ws2812_init(void);
void ws2812_demo(void);
int ws2812_flush(void);
void ws2812_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H_ */
