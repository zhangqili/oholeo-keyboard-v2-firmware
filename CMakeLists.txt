cmake_minimum_required(VERSION 3.13)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_PROJECT_NAME oholeo-keyboard-v2-firmware)

set(BOARD_SEARCH_PATH ${CMAKE_SOURCE_DIR}/boards)
set(ENV{HPM_SDK_BASE} ${CMAKE_SOURCE_DIR}/hpm_sdk)
set(ENV{GNURISCV_TOOLCHAIN_PATH} /home/xq123/Public/rv32imac_zicsr_zifencei_multilib_b_ext-linux/)
#set(ENV{HPM_SDK_TOOLCHAIN_VARIANT} nds-gcc)
set(BOARD_SEARCH_PATH ${CMAKE_SOURCE_DIR}/boards)
set(CMAKE_BUILD_TYPE release)
set(HPM_BUILD_TYPE "flash_xip")
set(BOARD oholeo_keyboard_v2)
set(RV_ARCH rv32imafdcv_zicsr_zifencei)
set(RV_ABI ilp32d)
#set(HPM_SDK_LD_NO_NANO_SPECS 1)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_HID 1)

# 查找并配置 HPM SDK
find_package(hpm-sdk REQUIRED HINTS ${CMAKE_SOURCE_DIR}/hpm_sdk)

# 定义项目
project(${CMAKE_PROJECT_NAME})

set(LIBAMP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
add_subdirectory_ifndef(WTF lib/libamp)

file(GLOB_RECURSE MY_SOURCES
    "./src/*.c")
# 添加源文件
sdk_app_src(${MY_SOURCES})

sdk_inc(
    include
    ${CMAKE_SOURCE_DIR}/lib/libamp/src
    ${CMAKE_SOURCE_DIR}/lib/libamp/usb
    ${CMAKE_SOURCE_DIR}/lib/libamp/src/midi
    ${CMAKE_SOURCE_DIR}/lib/libamp/src/midi/bytequeue
    ${CMAKE_SOURCE_DIR}/lib/libamp/src/lamp_array
    ${CMAKE_SOURCE_DIR}/lib/libamp/lib/littlefs
    ${CMAKE_SOURCE_DIR}/lib/libamp/lib/lufa
    ${CMAKE_SOURCE_DIR}/lib/libamp/lib/mquickjs
    ${CMAKE_BINARY_DIR}/lib/libamp
)

sdk_link_libraries(
    lib/libamp
)

