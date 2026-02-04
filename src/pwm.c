#include "pwm.h"
#include "board.h"
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"

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