/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_debug_console.h"
#include "keyboard.h"
#include "usbd_user.h"
#include "usb_config.h"
#include "adc.h"
#include "gptmr.h"
#include "analog.h"
#include "rgb.h"


int main(void)
{   
    board_init();
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOY, 0, gpiom_core0_fast);
    gpio_set_pin_output(HPM_FGPIO, GPIO_OE_GPIOY, 0);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOY, 1, gpiom_core0_fast);
    gpio_set_pin_output(HPM_FGPIO, GPIO_OE_GPIOY, 1);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOY, 2, gpiom_core0_fast);
    gpio_set_pin_output(HPM_FGPIO, GPIO_OE_GPIOY, 2);
    analog_channel_select(0);
    printf("hello world\n");

    /* ADC pin initialization */
    board_init_adc16_pins();

    /* ADC clock initialization */
    board_init_adc_clock(HPM_ADC0, true);
    board_init_adc_clock(HPM_ADC1, true);
    
    adc_init();
    gptmr_init();

    board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 3);
    gptmr_start_counter(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
    keyboard_init();
    keyboard_reset_to_default();
    board_delay_ms(100);

    filter_reset();
    analog_reset_range();
    analog_scan();
    usb_init();
    gptmr_start_counter(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
    g_keyboard_config.enable_report = true;
    while (1) {
      //rgb_update();
      board_delay_ms(1);
      //keyboard_task();
      //printf("%ld\n",g_keyboard_tick);
      AdvancedKey * key = &g_keyboard_advanced_keys[0];
      printf("%.2f\t%.2f\t%.2f\t%d\n", ringbuf_avg(&g_adc_ringbufs[g_analog_map[0]]), key->raw, key->value, key->key.state);
    }


    //printf("hello world\n");
    //keyboard_init();
    //
    //while(1)
    //{
    //    keyboard_task();
    //    u = getchar();
    //    if (u == '\r') {
    //        u = '\n';
    //    }
    //    printf("%c", u);
    //}
    return 0;
}
