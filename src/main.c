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
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_adc16_drv.h"
#include "keyboard.h"
#include "usbd_user.h"
#include "usb_config.h"
#include "adc.h"
#include "gptmr.h"
#include "analog.h"
#include "rgb.h"

uint32_t pulse_counter = 0;
bool beep_switch;
bool em_switch;

static void key_down_cb(void * k)
{
  UNUSED(k);
  pulse_counter=PULSE_LEN_MS;
}

void reset_pwm_counter(void)
{
    pwm_enable_reload_at_synci(HPM_PWM0);
    trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM0_SYNCI, 1);
    trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM0_SYNCI, 0);
}
void beep_init(void)
{
    pwm_config_t pwm_config = {0};
    pwm_cmp_config_t cmp_config = {0};
    
    uint32_t pwm_clk_freq = clock_get_frequency(clock_mot0);
    
    uint32_t target_freq = 1000;
    uint32_t reload_val = (pwm_clk_freq / target_freq) - 1;

    pwm_stop_counter(HPM_PWM0);
    reset_pwm_counter();
    pwm_get_default_pwm_config(HPM_PWM0, &pwm_config);

    pwm_config.enable_output = true;           // 开启输出
    pwm_config.dead_zone_in_half_cycle = 0;    // 无死区
    pwm_config.invert_output = false;          // 不反相
    pwm_set_reload(HPM_PWM0, 0, reload_val);
    pwm_set_start_count(HPM_PWM0, 0, 0);

    cmp_config.mode = pwm_cmp_mode_output_compare;
    cmp_config.cmp = (reload_val + 1) / 2;
    cmp_config.update_trigger = pwm_shadow_register_update_on_shlk;
    uint8_t cmp_index = 0; 
    if (status_success != pwm_setup_waveform(HPM_PWM0, 0, &pwm_config, cmp_index, &cmp_config, 1)) {
        printf("PWM Setup Failed!\n");
        return;
    }
    pwm_issue_shadow_register_lock_event(HPM_PWM0);
    //pwm_start_counter(HPM_PWM0);
    //board_delay_ms(100);
    //pwm_stop_counter(HPM_PWM0);
    //pwm_disable_output(HPM_PWM0, 0); 
}

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

    init_pwm_pins(HPM_PWM0);
    beep_init();
    //pwm_start_counter(HPM_PWM0);
    /* ADC pin initialization */
    board_init_adc16_pins();

    /* ADC clock initialization */
    board_init_adc_clock(HPM_ADC0, true);
    board_init_adc_clock(HPM_ADC1, true);
    
    adc_init();
    gptmr_init();
    extern int flash_init(void);
    flash_init();

    board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 3);
    gptmr_start_counter(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
    keyboard_init();
    for (uint8_t i = 0; i < TOTAL_KEY_NUM; i++)
    {
      key_attach(keyboard_get_key(i),KEY_EVENT_DOWN,key_down_cb);
    }
    keyboard_reset_to_default();
    board_delay_ms(100);

    filter_reset();
    analog_reset_range();
    analog_scan();
    usb_init();
    gptmr_start_counter(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
    g_keyboard_config.enable_report = true;
    beep_switch = true;

    while (1) {
      //rgb_update();
      board_delay_ms(1);
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

void keyboard_tick_task(void)
{
    g_keyboard_tick++;
    keyboard_task();
    if (pulse_counter)
    {
      pulse_counter--;
      if (beep_switch)
      {
        pwm_start_counter(HPM_PWM0);
      }
      if (em_switch)
      {
        //LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
      }
    }
    else
    {
        pwm_stop_counter(HPM_PWM0);
    }
}


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
