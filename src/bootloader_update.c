/*
 * Copyright (c) 2026 Zhangqi Li (@zhangqili)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "bootloader_update.h"

#include "board.h"
#include "hpm_csr_regs.h"
#include "hpm_interrupt.h"
#include "hpm_pdgo_drv.h"
#include "hpm_ppor_drv.h"
#include "hpm_romapi.h"

#define BOOTLOADER_FLASH_OFFSET       (0U)
#define BOOTLOADER_IMAGE_FLASH_OFFSET (0x400U)
#define BOOTLOADER_PROGRAM_CHUNK_SIZE (4096U)
#define BOOTLOADER_VERIFY_CHUNK_SIZE  (256U)

extern const uint8_t bootloader_image[];
extern const uint32_t bootloader_image_size;

static xpi_nor_config_t s_bootloader_nor_config;

ATTR_RAMFUNC static hpm_stat_t bootloader_program_chunk(uint32_t offset, uint32_t length)
{
    uint32_t irq_state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t status = rom_xpi_nor_program(
        BOARD_APP_XPI_NOR_XPI_BASE,
        xpi_xfer_channel_auto,
        &s_bootloader_nor_config,
        (const uint32_t *)(bootloader_image + offset),
        BOOTLOADER_IMAGE_FLASH_OFFSET + offset,
        length);
    restore_global_irq(irq_state);
    return status;
}

ATTR_RAMFUNC static hpm_stat_t bootloader_erase_reserved_region(void)
{
    uint32_t irq_state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t status = rom_xpi_nor_erase(
        BOARD_APP_XPI_NOR_XPI_BASE,
        xpi_xfer_channel_auto,
        &s_bootloader_nor_config,
        BOOTLOADER_FLASH_OFFSET,
        BOARD_BOOTLOADER_RESERVED_LENGTH);
    restore_global_irq(irq_state);
    return status;
}

ATTR_RAMFUNC static hpm_stat_t bootloader_verify_image(void)
{
    uint32_t verify_buffer[BOOTLOADER_VERIFY_CHUNK_SIZE / sizeof(uint32_t)];

    for (uint32_t offset = 0; offset < bootloader_image_size;) {
        uint32_t length = bootloader_image_size - offset;
        if (length > sizeof(verify_buffer)) {
            length = sizeof(verify_buffer);
        }

        hpm_stat_t status = rom_xpi_nor_read(
            BOARD_APP_XPI_NOR_XPI_BASE,
            xpi_xfer_channel_auto,
            &s_bootloader_nor_config,
            verify_buffer,
            BOOTLOADER_IMAGE_FLASH_OFFSET + offset,
            length);
        if (status != status_success) {
            return status;
        }

        const uint8_t *actual = (const uint8_t *)verify_buffer;
        for (uint32_t index = 0; index < length; index++) {
            if (actual[index] != bootloader_image[offset + index]) {
                return status_fail;
            }
        }
        offset += length;
    }

    return status_success;
}

ATTR_RAMFUNC __attribute__((used))
hpm_stat_t keyboard_flash_bootloader_and_reboot(void)
{
    if ((bootloader_image_size == 0U)
        || (bootloader_image_size
            > (BOARD_BOOTLOADER_RESERVED_LENGTH - BOOTLOADER_IMAGE_FLASH_OFFSET))
        || ((bootloader_image_size % sizeof(uint32_t)) != 0U)) {
        return status_invalid_argument;
    }

    xpi_nor_config_option_t option = {0};
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    hpm_stat_t status = rom_xpi_nor_auto_config(
        BOARD_APP_XPI_NOR_XPI_BASE,
        &s_bootloader_nor_config,
        &option);
    if (status != status_success) {
        return status;
    }

    status = bootloader_erase_reserved_region();
    if (status != status_success) {
        return status;
    }

    for (uint32_t offset = 0; offset < bootloader_image_size;) {
        uint32_t length = bootloader_image_size - offset;
        if (length > BOOTLOADER_PROGRAM_CHUNK_SIZE) {
            length = BOOTLOADER_PROGRAM_CHUNK_SIZE;
        }

        status = bootloader_program_chunk(offset, length);
        if (status != status_success) {
            return status;
        }
        offset += length;
    }

    status = bootloader_verify_image();
    if (status != status_success) {
        return status;
    }

    /* Ensure a previous bootloader request cannot force DFU mode. */
    pdgo_write_gpr(HPM_PDGO, 0, 0);
    ppor_sw_reset(HPM_PPOR, 10);

    return status_success;
}
