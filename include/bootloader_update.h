/*
 * Copyright (c) 2026 Zhangqi Li (@zhangqili)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef BOOTLOADER_UPDATE_H_
#define BOOTLOADER_UPDATE_H_

#include "hpm_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Erase and program the embedded Bootloader, verify it, then reboot normally.
 *
 * The function returns an error without resetting if any validation, erase,
 * program, or verification step fails.  On success the reset request normally
 * prevents the function from returning.
 */
hpm_stat_t keyboard_flash_bootloader_and_reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_UPDATE_H_ */
