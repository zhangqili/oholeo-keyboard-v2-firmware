/*
 * Standard USB DFU support for the TinyUF2 bootloader.
 *
 * The DFU image is the application's flash_uf2 .bin, not its UF2 container.
 * The first word is BOARD_UF2_SIGNATURE. It is deliberately committed only
 * after the complete image has reached flash so an interrupted transfer cannot
 * be booted as an application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_api.h"
#include "tusb.h"
#include "dfu.h"

static bool s_download_started;
static bool s_download_failed;
static bool s_signature_valid;
static bool s_reset_pending;
static uint32_t s_downloaded_bytes;

static void dfu_reset_download_state(void)
{
    s_download_started = false;
    s_download_failed = false;
    s_signature_valid = false;
    s_downloaded_bytes = 0;
}

static void dfu_finish_with_error(uint8_t status)
{
    s_download_failed = true;
    uf2_board_flash_abort();
    indicator_set(STATE_WRITING_FINISHED);
    tud_dfu_finish_flashing(status);
}

void dfu_init(void)
{
    dfu_reset_download_state();
    s_reset_pending = false;
}

void dfu_task(void)
{
    if (s_reset_pending) {
        s_reset_pending = false;
        uf2_board_dfu_complete();
    }
}

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
    (void) alt;

    /* NOR erase/program operations are synchronous in this bootloader. */
    return (state == DFU_MANIFEST) ? 0U : 0U;
}

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length)
{
    uint32_t offset;
    uint32_t address;

    if (alt != 0U) {
        dfu_finish_with_error(DFU_STATUS_ERR_TARGET);
        return;
    }

    /* A block zero starts a fresh DFU image, including after CLRSTATUS. */
    if (block_num == 0U) {
        uf2_board_flash_abort();
        dfu_reset_download_state();
    }

    offset = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;
    if ((offset != s_downloaded_bytes) || (length == 0U)
        || (offset > uf2_board_flash_size())
        || (length > (uf2_board_flash_size() - offset))) {
        dfu_finish_with_error(DFU_STATUS_ERR_ADDRESS);
        return;
    }

    if (block_num == 0U) {
        uint32_t signature;

        if ((length < sizeof(signature))) {
            dfu_finish_with_error(DFU_STATUS_ERR_FILE);
            return;
        }

        memcpy(&signature, data, sizeof(signature));
        if (signature != BOARD_UF2_SIGNATURE) {
            dfu_finish_with_error(DFU_STATUS_ERR_FILE);
            return;
        }

        s_download_started = true;
        s_signature_valid = true;
        indicator_set(STATE_WRITING_STARTED);
        uf2_board_flash_invalidate_app();

        /* Keep the first word invalid until manifest succeeds. */
        address = BOARD_FLASH_APP_START + sizeof(signature);
        uf2_board_flash_write(address, data + sizeof(signature), length - sizeof(signature));
    } else {
        if (!s_download_started) {
            dfu_finish_with_error(DFU_STATUS_ERR_NOTDONE);
            return;
        }

        address = BOARD_FLASH_APP_START + offset;
        uf2_board_flash_write(address, data, length);
    }

    s_downloaded_bytes += length;
    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

void tud_dfu_manifest_cb(uint8_t alt)
{
    const uint32_t signature = BOARD_UF2_SIGNATURE;

    if ((alt != 0U) || !s_download_started || s_download_failed
        || !s_signature_valid || (s_downloaded_bytes <= sizeof(signature))) {
        dfu_finish_with_error(DFU_STATUS_ERR_FIRMWARE);
        return;
    }

    /* Commit the image's boot-valid marker only after every data block is flushed. */
    uf2_board_flash_flush();
    uf2_board_flash_write(BOARD_FLASH_APP_START, &signature, sizeof(signature));
    uf2_board_flash_flush();

    indicator_set(STATE_WRITING_FINISHED);
    tud_dfu_finish_flashing(DFU_STATUS_OK);
    s_reset_pending = true;
}

uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t *data, uint16_t length)
{
    uint32_t const offset = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;
    uint32_t const image_size = uf2_board_flash_size();
    uint32_t bytes_to_read;

    if ((alt != 0U) || (offset >= image_size)) {
        /* A zero-length response terminates the host's firmware upload. */
        return 0;
    }

    bytes_to_read = image_size - offset;
    if (bytes_to_read > length) {
        bytes_to_read = length;
    }

    /*
     * The current UF2 application format has no persistent image-length
     * field, so expose the complete application partition. The bytes after
     * the application are normally erased (0xff) and retaining them makes
     * the uploaded binary directly suitable for a full-partition restore.
     */
    uf2_board_flash_read(BOARD_FLASH_APP_START + offset, data, bytes_to_read);
    return (uint16_t) bytes_to_read;
}

void tud_dfu_abort_cb(uint8_t alt)
{
    (void) alt;
    uf2_board_flash_abort();
    dfu_reset_download_state();
    indicator_set(STATE_WRITING_FINISHED);
}

void tud_dfu_detach_cb(void)
{
    tud_dfu_abort_cb(0);
}
