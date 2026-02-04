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
#include "hpm_qeiv2_drv.h"
#include "keyboard.h"
#include "usbd_user.h"
#include "usb_config.h"
#include "adc.h"
#include "gptmr.h"
#include "analog.h"
#include "rgb.h"
#include "ws2812.h"


/* Static function declaration */
static void qeiv2_ec11_init(void)
{
    qeiv2_mode_config_t mode_config = {0};

    mode_config.work_mode = qeiv2_work_mode_abz;
    mode_config.phcnt_max = 0xFFFFFFFF;  // 无限旋转
    mode_config.z_count_inc_mode = qeiv2_z_count_inc_on_phase_count_max;
    mode_config.z_cali_enable = false;
    mode_config.z_cali_ignore_ab = false;
    mode_config.phcnt_idx = 0;

    qeiv2_config_mode(BOARD_BLDC_QEIV2_BASE, &mode_config);
    qeiv2_set_phase_cnt(BOARD_BLDC_QEIV2_BASE, 0);

    /* 2. 配置输入滤波器 (Filter) - 适配您的函数原型 */
    
    /* * 计算滤波长度 (filtlen)
     * 根据您提供的函数逻辑，它会自动计算 shift 和 len。
     * 只要传入我们期望的总时钟周期数即可。
     * * 假设 Motor Clock = 100MHz (100,000,000 Hz)
     * 机械抖动通常在 1ms - 5ms。
     * 1ms 对应的周期数 = 100,000,000 * 0.001 = 100,000
     */
    uint32_t motor_clk = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
    uint32_t filter_cycles = motor_clk / 1000; // 约 1ms 去抖时间

    /* * 调用 qeiv2_config_filter
     * 参数顺序: base, phase, outinv, mode, sync, filtlen
     */
     
    qeiv2_config_filter(BOARD_BLDC_QEIV2_BASE, 
                        qeiv2_filter_phase_a, // 相位 A
                        false,                // outinv: 不反相
                        qeiv2_filter_mode_delay, // mode: 延时滤波模式 (推荐用于去抖)
                        false,                // sync: 异步输出
                        filter_cycles         // filtlen: 滤波长度
                        );

    qeiv2_config_filter(BOARD_BLDC_QEIV2_BASE, 
                        qeiv2_filter_phase_b, // 相位 B
                        false,                // outinv: 不反相
                        qeiv2_filter_mode_delay, // mode: 延时滤波模式
                        false,                // sync: 异步输出
                        filter_cycles         // filtlen: 滤波长度
                        );

    qeiv2_disable_irq(BOARD_BLDC_QEIV2_BASE, 0xFFFFFFFF); 
    qeiv2_clear_status(BOARD_BLDC_QEIV2_BASE, 0xFFFFFFFF);
}

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
    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_GPIO_A_09;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, 9, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOA, 9);
    gpio_disable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOA, 9);

    
    printf("hello world\n");
    ws2812_init();
    ws2812_flush();
    init_qeiv2_ab_pins(BOARD_BLDC_QEIV2_BASE);

    qeiv2_ec11_init();

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
    g_keyboard_config.enable_report = false;
    for (uint8_t i = 0; i < TOTAL_KEY_NUM; i++)
    {
      key_attach(keyboard_get_key(i),KEY_EVENT_DOWN,key_down_cb);
    }
    keyboard_reset_to_default();
    gptmr_start_counter(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
    //board_delay_ms(100);

    rgb_init_flash();
    filter_reset();
    analog_reset_range();
    analog_scan();
    usb_init();
    g_keyboard_config.enable_report = true;
    beep_switch = false;
    g_keyboard_advanced_keys[15].config.mode = ADVANCED_KEY_DIGITAL_MODE;

    while (1) {
      static bool rgb_state;
      static bool last_rgb_state;
      static bool flush_failed;
      rgb_update();
      //if ((g_keyboard_tick%8000) < 7950)
      //{
      //  for (int i = 0; i < RGB_NUM; i++)
      //  {
      //    rgb_set(i, 0xFF, 0xFF, 0xFF);
      //  }
      //  rgb_state = true;
      //}
      //else
      //{
      //  for (int i = 0; i < RGB_NUM; i++)
      //  {
      //    rgb_set(i, 0, 0, 0);
      //  }
      //  rgb_state = false;
      //}
      if (rgb_state || (last_rgb_state && !rgb_state) || flush_failed)
      {
        flush_failed = ws2812_flush();
      }
      last_rgb_state = rgb_state;
      AdvancedKey * key = &g_keyboard_advanced_keys[57];
      //printf("%.2f,%.2f,%.2f,%.2f,%d\n",ringbuf_avg(&g_adc_ringbufs[g_analog_map[16]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[17]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[28]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[35]]), rgb_state);
      printf("%.2f,%.2f,%.2f,%.2f,%d\n",g_keyboard_advanced_keys[16].value, g_keyboard_advanced_keys[17].value, g_keyboard_advanced_keys[28].value, g_keyboard_advanced_keys[35].value, rgb_state);
      board_delay_ms(1);
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
