/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_debug_console.h"
#include "keyboard.h"

#define LED_FLASH_PERIOD_IN_MS 300

int main(void)
{
    int u;
    board_init();
    //board_init_led_pins();

    //board_timer_create(LED_FLASH_PERIOD_IN_MS, board_led_toggle);

    printf("hello world\n");
    while(1)
    {
        u = getchar();
        if (u == '\r') {
            u = '\n';
        }
        printf("%c", u);
    }
    
    board_init_usb((USB_Type *)HPM_USB0_BASE);
    //board_init_gpio_pins();
    //gpio_set_pin_input(BOARD_APP_GPIO_CTRL, BOARD_APP_GPIO_INDEX, BOARD_APP_GPIO_PIN);

    intc_set_irq_priority(IRQn_USB0, 2);
    //board_timer_create(LED_FLASH_PERIOD_IN_MS, board_led_toggle);

    printf("cherry usb hid_keyboard device sample.\n");

    hid_keyboard_init(0, HPM_USB0_BASE);
    while (1) {
        hid_keyboard_test(0);
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
