cmake_minimum_required(VERSION 3.13)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_PROJECT_NAME oholeo-keyboard-v2-firmware)

set(APP_NAME "oholeo-keyboard-v2-bootloader")
set(BOARD_SEARCH_PATH ${CMAKE_SOURCE_DIR}/../boards)
set(ENV{HPM_SDK_BASE} ${CMAKE_SOURCE_DIR}/../hpm_sdk)
set(ENV{GNURISCV_TOOLCHAIN_PATH} /home/xq123/Public/rv32imac_zicsr_zifencei_multilib_b_ext-linux/)
#set(ENV{HPM_SDK_TOOLCHAIN_VARIANT} nds-gcc)
set(CMAKE_BUILD_TYPE release)
set(HPM_BUILD_TYPE "flash_xip")
set(BOARD oholeo_keyboard_v2)
set(RV_ARCH rv32imafdcv_zicsr_zifencei)
set(RV_ABI ilp32d)
#set(HPM_SDK_LD_NO_NANO_SPECS 1)

set(CONFIG_TINYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_MSC 1)
set(CONFIG_USB_DEVICE_HID 1)

# 查找并配置 HPM SDK
find_package(hpm-sdk REQUIRED HINTS ${CMAKE_SOURCE_DIR}/../hpm_sdk)

# 定义项目
project(tinyuf2_bootloader)

sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)

sdk_inc(src)

sdk_app_src(src/main.c)

sdk_app_src(src/ghostfat.c)
sdk_app_src(src/msc.c)
sdk_app_src(src/usb_descriptors.c)
sdk_app_src(src/board_api.c)
generate_ide_projects()
