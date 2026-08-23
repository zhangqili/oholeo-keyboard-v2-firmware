.. _tinyuf2:

TinyUF2
==============

概述
------

TinyUF2这个示例提供了一个UF2 bootloader。

* bootloader模式，运行起来之后，将在PC端枚举为一个u盘。这时可以将

构建在UF2容器中的一个应用，通过拖拽将其放入枚举到的u盘中。TinyUF2将把该容器
内的应用烧写进板载的flash中，烧写完成之后，将直接跳转至刚刚写入的应用。

* 启动模式, 可以通过按住TinyUF2按键(请确认具体开发板 {ref}`按键 <board_resource>` 部分描述)同时reset板子，进入bootloader模式，也可以直接跳转到已经通过TinyUF2

写入的flash镜像，若无镜像，将直接退回到bootloader模式。

UF2容器打包
~~~~~~~~~~~~~~~

一个可以被用作TinyUF2引导的应用需要满足以下条件：
1. 该应用应该使用SDK中的flash_uf2或者flash_sdram_uf2链接脚本链接
1. 运行一下命令打包生成UF2容器 （假设应用二进制文件名为hello_world.bin）:


.. code-block:: python

   python3 uf2conv.py -f 0x0A4D5048 -b 0x80020000 -c -o update.uf2 hello_world.bin

该命令把hello_world.bin（从0x80020000启动）打包生成update.uf2这个UF2容器文件。
0x0A4D5048: 先楫半导体UF2的家族ID，需要和uf2conv.py匹配使用

硬件设置
------------

无特殊设置

运行现象
------------

* 在启动模式下，如果有合法镜像已经写入正确位置，它会跳转至该应用，串口终端会输出如下信息：


.. code-block:: console

   TinyUF2
   Jump to application @0x80020004(0x800291b7)

注:
* 0x80020004 是写入的应用的起始地址

* 0x800291b7 是写入的应用起始地址上的指令

* 在bootloader模式下，如果一切正常，串口终端会输出如下信息：


.. code-block:: console

   TinyUF2
   Starting DFU mode

此时可以插入USB线，在PC端将打包的UF2容器文件拖入枚举出的u盘内。烧写完成后，它会直接跳转
到该应用。

在跳转到有效应用之前，bootloader 会通过 MUX 通道 6 和 ADC1 通道 15 对按键 0 进行 8 次
有效采样并计算平均值。平均值小于 ``8192``、大于 ``57344``，或 ADC 初始化/采样失败时，
设备会留在待烧录模式。待烧录期间，无论 USB 是否已经连接，全部 69 颗灯珠均显示亮度为
``100`` 的红色；开始写入固件后切换为橙色。正常跳转应用时不会初始化或刷新灯珠。

USB DFU 升级
--------------

bootloader 同时提供标准 USB DFU 接口和现有的 UF2 U 盘升级接口。DFU 应使用通过
``flash_uf2`` 链接脚本构建出的原始应用 ``.bin`` 文件，不能使用 ``.uf2`` 容器。例如：

在 Windows 10/11 上，bootloader 会通过 Microsoft OS 2.0 描述符仅将 DFU 接口声明为
兼容 WinUSB。系统首次连接时会自动加载内置的 ``winusb.sys``，无需安装自定义 INF 或使用
Zadig；UF2 U 盘和 HID 接口仍使用各自的系统类驱动。

.. code-block:: console

   dfu-util -a 0 -D build/output/oholeo-keyboard-v2.bin

DFU 开始时会先使当前应用失效，只有完整传输和写入成功后才恢复启动签名。因此升级过程中断电或
取消升级时，设备会继续停留在 bootloader，而不会尝试启动不完整的应用。

DFU 的 ``Firmware Upload``（从设备读取）也已启用，可将应用分区备份到主机。由于当前 UF2
应用格式没有保存实际镜像长度，导出的内容为完整应用分区；尾部通常是擦除状态的 ``0xff`` 填充。
请将其视为原始备份镜像并妥善保管，其中包含设备当前安装的固件。
