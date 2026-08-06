/*
 * Standard USB DFU support for the TinyUF2 bootloader.
 */
#ifndef BOOTLOADER_DFU_H
#define BOOTLOADER_DFU_H

void dfu_init(void);
void dfu_task(void);

#endif /* BOOTLOADER_DFU_H */
