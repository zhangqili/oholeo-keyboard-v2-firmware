#include "qei.h"
#include "board.h"
#include "hpm_qeiv2_drv.h"

/* Static function declaration */
void qeiv2_ec11_init(void)
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