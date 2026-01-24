cmake_minimum_required(VERSION 3.13)

set(ENV{HPM_SDK_BASE} ${CMAKE_SOURCE_DIR}/hpm_sdk)
set(ENV{GNURISCV_TOOLCHAIN_PATH} /home/xq123/Public/rv32imac_zicsr_zifencei_multilib_b_ext-linux/)
#set(ENV{HPM_SDK_TOOLCHAIN_VARIANT} nds-gcc)
set(BOARD_SEARCH_PATH ${CMAKE_SOURCE_DIR}/boards)
set(CMAKE_BUILD_TYPE release)
set(HPM_BUILD_TYPE "flash_xip")
set(BOARD hpm5300evk)
set(RV_ARCH rv32imafdcv_zicsr_zifencei)
set(RV_ABI ilp32d)
#set(HPM_SDK_LD_NO_NANO_SPECS 1)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_HID 1)
# 查找并配置 HPM SDK
find_package(hpm-sdk REQUIRED HINTS ${CMAKE_SOURCE_DIR}/hpm_sdk)

# 定义项目
project(oholeo_keyboard_v2)

# 添加源文件
sdk_app_src(src/main.c)

sdk_inc(include)
