# Oholeo Keyboard V2

硬件工程: <>

配置器: <https://github.com/zhangqili/EMIKeyboardConfigurator>

## 特性
+ 支持传统触发、快速触发(Rapid Trigger)、差速触发三种模式
+ 支持Mod tap、Mutex(类似Rappy Snappy)、DKS等高级按键
+ 支持绑定键盘，鼠标，媒体控制，系统控制按键
+ 支持板载Flash切换配置
+ 支持最多5层按键绑定
+ 支持切换6键无冲与全键无冲模式
+ 支持模拟游戏摇杆与按键，可通过Steam重映射
+ 支持模拟MIDI键盘
+ 支持低延迟模式，该模式会关闭大部分灯效
+ 支持硬件录制播放按键宏
+ 配套免驱配置器
+ 支持Windows动态光效(实验性，需要在keyboard_conf.h中取消注释LIGHTING_ENABLE，可能会干扰ADC采样)
+ 支持调用脚本(实验性，需要在keyboard_conf.h中取消注释SCRIPT_ENABLE)

## TODO
+ 免映射模拟游戏手柄
+ 使用蜂鸣器播放MIDI音符


### 前置条件
- Python
- PyYAML模块```pip install pyyaml```
- CMake
- Ninja
- HPMicro提供的工具链\
(Windows:<https://gitee.com/hpmicro/sdk_env>
Linux:<https://github.com/riscv-collab/riscv-gnu-toolchain>)
- HPM Manufacturing Tool<https://github.com/hpmicro/hpm_manufacturing_tool>
- 将该仓库克隆到本地并在该仓库文件夹打开命令行终端
```PowerShell
git clone --recurse-submodules https://github.com/zhangqili/oholeo-keyboard-v2-firmware.git
cd oholeo-keyboard-v2-firmware
```

### 修改CMakeLists.txt
将该行设为工具链所在目录
```cmake
set(ENV{GNURISCV_TOOLCHAIN_PATH} /home/xq123/Public/rv32imac_zicsr_zifencei_multilib_b_ext-linux/)
```

### 运行编译命令
```PowerShell
mkdir build
cd build
cmake .. -G"Ninja"
ninja
```

如果编译成功，会看到类似以下的结果，编译生成的二进制文件oholeo-keyboard-v2.bin位于output文件夹
```
[120/120] Linking C executable output/oholeo-keyboard-v2.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:      404564 B         1 MB     38.58%
             ILM:        5208 B       128 KB      3.97%
             DLM:      101904 B     130304 B     78.20%
        AHB_SRAM:          0 GB        32 KB      0.00%
ninja  10.43s user 1.47s system 211% cpu 5.631 total
```

## 烧录
未烧录固件时，按住BOOT键上电或上电后按住BOOT键时点按一下NRST键进入Bootloder。\
烧录固件后，键盘正常运行时按住BOOT键进入Bootloader。\
将USB转UART连接至对应的TX和RX孔。\
在HPM Manufacturing Tool界面，芯片选择HPM5300，类型选择UART，选择对应的串口，选择编译得到的二进制文件，连接并烧写。


## 使用说明

### 上电至灯效结束后

|按键|功能|
|---|---|
|`FN`|清除所有数据，恢复出厂设置并重启|
|`Backspace`|恢复默认配置并重启|


### 默认快捷键

|按键|功能|
|---|---|
|`FN`+`DEL`+`D`|进入Debug状态|
|`FN`+`DEL`+`B`|切换使能蜂鸣器|
|`FN`+`DEL`+`M`|切换使能电磁阀|
|`FN`+`DEL`+`N`|恢复默认状态，取消使能蜂鸣器，取消使能电磁阀，关闭低延迟模式，开启全键无冲|
|`FN`+`DEL`+`R`|重启|
|`FN`+`DEL`+`F`|清除所有数据并恢复出厂设置|
|`FN`+`DEL`+`S`|将当前配置写入至Flash|
|`FN`+`DEL`+`L`|切换低延迟模式|
|`FN`+`DEL`+`T`|切换全键无冲与六键无冲模式|
|`FN`+`DEL`+`Esc`|进入Bootloader|
|`FN`+`DEL`+`Backspace`|恢复默认配置|
|`FN`+`DEL`+`1`|使用配置文件0|
|`FN`+`DEL`+`2`|使用配置文件1|
|`FN`+`DEL`+`3`|使用配置文件2|
|`FN`+`DEL`+`4`|使用配置文件3|
|`FN`+`DEL`+`Win`|切换锁定Win键|
