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
#include "hpm_spi_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_serial_nor.h"
#include "hpm_serial_nor_host_port.h"
#include "hpm_mchtmr_drv.h"
#include "rvbacktrace.h"

#include "keyboard.h"
#include "usbd_user.h"
#include "usb_config.h"
#include "adc.h"
#include "gptmr.h"
#include "analog.h"
#include "rgb.h"
#include "ws2812.h"
#include "pwm.h"
#include "qei.h"
#include "driver.h"

uint32_t pulse_counter = 0;
bool beep_switch;
bool em_switch;
static uint32_t timer_freq_in_hz;
hpm_serial_nor_t nor_flash_dev = {0};
hpm_serial_nor_info_t flash_info;
uint32_t debug;
uint32_t debug1;
volatile uint64_t start_time;
volatile uint64_t end_time1;
volatile uint64_t end_time;
volatile uint32_t err_cnt;

static void key_down_cb(void * k)
{
  UNUSED(k);
  pulse_counter=PULSE_LEN_MS;
}
volatile bool is_init_complete = false;

int main(void)
{   
  board_init();
  board_ungate_mchtmr_at_lp_mode();
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
  HPM_IOC->PAD[IOC_PAD_PA03].FUNC_CTL = IOC_PA03_FUNC_CTL_GPIO_A_03;
  gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, 3, gpiom_soc_gpio0);
  gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOA, 3);
  gpio_disable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOA, 3);
  serial_nor_get_board_host(&nor_flash_dev.host);
  board_init_spi_clock(nor_flash_dev.host.host_param.param.host_base);
  serial_nor_spi_pins_init(nor_flash_dev.host.host_param.param.host_base);
  timer_freq_in_hz = clock_get_frequency(clock_mchtmr0);
  hpm_serial_nor_init(&nor_flash_dev, &flash_info);
  if (hpm_serial_nor_get_info(&nor_flash_dev, &flash_info) == status_success) {
      printf("the flash sfdp version:%d\n", flash_info.sfdp_version);
      printf("the flash size:%d KB\n", flash_info.size_in_kbytes);
      printf("the flash page_size:%d Byte\n", flash_info.page_size);
      printf("the flash sector_size:%d KB\n", flash_info.sector_size_kbytes);
      printf("the flash block_size:%d KB\n", flash_info.block_size_kbytes);
      printf("the flash sector_erase_cmd:0x%02x\n", flash_info.sector_erase_cmd);
      printf("the flash block_erase_cmd:0x%02x\n", flash_info.block_erase_cmd);
  }
  
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
  //extern int flash_init(void);
  //flash_init();
  board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);
  intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 7);
  gptmr_start_counter(RINGBUF_TICK_GPTMR, RINGBUF_TICK_GPTMR_CH);
  keyboard_init();
  g_keyboard_config.enable_report = false;
  for (uint8_t i = 0; i < TOTAL_KEY_NUM; i++)
  {
    key_attach(keyboard_get_key(i),KEY_EVENT_DOWN,key_down_cb);
  }
  //keyboard_reset_to_default();
  //board_delay_ms(100);
  gptmr_start_counter(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
  rgb_init_flash();
  filter_reset();
  analog_reset_range();
  analog_scan();
  
  if (ringbuf_avg(&g_adc_ringbufs[g_analog_map[0]])< 8192 || ringbuf_avg(&g_adc_ringbufs[g_analog_map[0]]) > (65536 - 8192))
  {
    for (uint8_t i = 0; i < ADVANCED_KEY_NUM; i++)
    {
        rgb_set(i, 100, 0, 0);
    }
    led_flush();
    keyboard_jump_to_bootloader();
  }
  if (ringbuf_avg(&g_adc_ringbufs[g_analog_map[13]]) < 8192 || ringbuf_avg(&g_adc_ringbufs[g_analog_map[13]]) > (65536 - 8192))
  {
    keyboard_reset_to_default();
    keyboard_save();
    keyboard_reboot();
  }
  if (ringbuf_avg(&g_adc_ringbufs[g_analog_map[14]]) < 8192 || ringbuf_avg(&g_adc_ringbufs[g_analog_map[14]]) > (65536 - 8192))
  {
    keyboard_reset_to_default();
    keyboard_save();
    keyboard_reboot();
  }
  if (ringbuf_avg(&g_adc_ringbufs[g_analog_map[65]]) < 8192 || ringbuf_avg(&g_adc_ringbufs[g_analog_map[65]]) > (65536 - 8192))
  {
    keyboard_factory_reset();
    keyboard_reboot();
  }
  keyboard_tick_timer_config();
  gptmr_start_counter(KEYBOARD_TICK_GPTMR, KEYBOARD_TICK_GPTMR_CH);
  usb_init();
  g_keyboard_config.enable_report = true;
  g_keyboard_config.nkro = true;
  is_init_complete = true;
  //beep_switch = true;
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
    if (gpio_read_pin(HPM_GPIO0, GPIO_OE_GPIOA, 3))
    {
      keyboard_reboot();
    }
    
    if (rgb_state || (last_rgb_state && !rgb_state) || flush_failed)
    {
      flush_failed = ws2812_flush();
    }
    last_rgb_state = rgb_state;
    AdvancedKey * key = &g_keyboard_advanced_keys[1];
    UNUSED(key);
    //printf("%ld\t%.0f\t%.2f\t%d\n", debug1, key->raw/16.0f, key->value, key->key.report_state);
    //printf("%ld\t%ld\t%d\t%ld\t%.0f\t%.0f\t%ld\t%ld\t%ld\n", g_keyboard_tick, debug1, err_cnt, g_keyboard_report_flags.keyboard, g_keyboard_advanced_keys[6].raw/16.0f, g_keyboard_advanced_keys[23].raw/16.0f, (uint32_t)start_time, (uint32_t)end_time1, (uint32_t)end_time);
    //printf("%.2f,%.2f,%.2f,%.2f,%d\n",ringbuf_avg(&g_adc_ringbufs[g_analog_map[16]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[17]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[28]]), ringbuf_avg(&g_adc_ringbufs[g_analog_map[35]]), rgb_state);
    printf("%ld\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\n",debug1 ,g_keyboard_advanced_keys[6].raw/16, g_keyboard_advanced_keys[23].raw/16, g_keyboard_advanced_keys[22].raw/16, g_keyboard_advanced_keys[24].raw/16, g_keyboard_advanced_keys[1].raw/16, g_keyboard_advanced_keys[0].raw/16, g_keyboard_advanced_keys[16].raw/16);
    //printf("%ld\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\t%.0f\n",debug1 ,g_keyboard_advanced_keys[44].raw/16, g_keyboard_advanced_keys[45].raw/16, g_keyboard_advanced_keys[46].raw/16, g_keyboard_advanced_keys[47].raw/16, g_keyboard_advanced_keys[1].raw/16, g_keyboard_advanced_keys[0].raw/16, g_keyboard_advanced_keys[16].raw/16);
    //printf("%ld\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",debug1 ,g_keyboard_advanced_keys[9].raw, g_keyboard_advanced_keys[10].raw, g_keyboard_advanced_keys[11].raw, g_keyboard_advanced_keys[12].raw, g_keyboard_advanced_keys[13].raw, g_keyboard_advanced_keys[14].raw, g_keyboard_advanced_keys[15].raw);
    //board_delay_ms(1);
  }
  return 0;
}

void rgb_update_callback()
{
  extern uint8_t g_current_profile_index;
  extern uint8_t g_current_layer;
  /*
  if (g_snake.running)
  {
    snake_move(&g_snake);
    draw_snake(&g_snake);
    return;
  }
  */
	if(g_keyboard_led_state.caps_lock)
  {
	  g_rgb_colors[g_rgb_inverse_mapping[30]].r = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[30]].g = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[30]].b = 0xff;//cap lock
	}
	if(g_keyboard_led_state.scroll_lock)
  {
	  g_rgb_colors[g_rgb_inverse_mapping[26]].r = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[26]].g = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[26]].b = 0xff;//cap lock
	}
  if (g_current_layer == 2)
  {
	  g_rgb_colors[g_rgb_inverse_mapping[0]].r = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[0]].g = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[0]].b = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[1]].r = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[1]].g = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[1]].b = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[2]].r = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[2]].g = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[2]].b = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[3]].r = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[3]].g = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[3]].b = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[4]].r = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[4]].g = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[4]].b = 0;
	  g_rgb_colors[g_rgb_inverse_mapping[g_current_profile_index+1]].r = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[g_current_profile_index+1]].g = 0xff;
	  g_rgb_colors[g_rgb_inverse_mapping[g_current_profile_index+1]].b = 0xff;
    if (g_keyboard_config.nkro)
    {
      g_rgb_colors[g_rgb_inverse_mapping[21]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[21]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[21]].b = 0xff;
    }
    if (g_keyboard_config.debug)
    {
      g_rgb_colors[g_rgb_inverse_mapping[33]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[33]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[33]].b = 0xff;
    }
    if (beep_switch)
    {
      g_rgb_colors[g_rgb_inverse_mapping[48]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[48]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[48]].b = 0xff;
    }
    if (em_switch)
    {
      g_rgb_colors[g_rgb_inverse_mapping[47]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[47]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[47]].b = 0xff;
    }
    /*
    if (low_latency_mode)
    {
      g_rgb_colors[g_rgb_inverse_mapping[37]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[37]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[37]].b = 0xff;
    }
    */
    if (g_keyboard_config.winlock)
    {
      g_rgb_colors[g_rgb_inverse_mapping[58]].r = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[58]].g = 0xff;
      g_rgb_colors[g_rgb_inverse_mapping[58]].b = 0xff;
    }
  }
}


void keyboard_tick_task(void)
{ 
  start_time = mchtmr_get_count(HPM_MCHTMR);
  g_keyboard_tick++;
  if (!(g_keyboard_tick%8000))
  {
    debug1 = debug;
    debug = 0;
  }
  
  if (!is_init_complete)
  {
    return;
  }
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
  end_time1 = mchtmr_get_count(HPM_MCHTMR);
}

long exception_handler(long cause, long epc)
{
    rvbacktrace_trap(&cause, &epc);
    while (1) {
    }
    return epc;
}
