# 串口数据接收实验

本实验的目标是通过串口读取模拟生成的 IMU 数据，并将其发布到 ROS 2 话题中。

## 依赖安装

> 在进行本实验前，应已按照招新文档的说明完成 Ubuntu 22.04 上 C++ 编译工具链及 ROS 2 Humble 的安装。

> 文仅列出使用 apt 包管理器的安装方式。如需采用其他方式安装，请参考各依赖的官方文档。

1. socat
```
$ sudo apt install socat
```

2. [Boost](https://www.boost.org/doc/user-guide/getting-started.html)
```
$ sudo apt install libboost-all-dev
```

3. [spdlog](https://github.com/gabime/spdlog)
```
$ sudo apt install libspdlog-dev
# fmt 库是 spdlog 的依赖
$ sudo apt install libfmt-dev
```

## 编译项目
```
$ mkdir build && cd build
$ cmake ..
$ make
```

## 实验配置步骤
1. 运行 scripts/start.sh，创建 socat 虚拟串口对：
```
$ cd scripts
$ sudo bash ./start.sh
```

2. 运行模拟数据发送程序：
```
# 如果使用笔者提供的 CMakePresets，选择 debug/release 作为 preset 时，编译产物会生成在 build/debug 或 build/release 目录下。
# 若未使用 preset，产物将生成在 build 的直接子目录下。
$ cd build

# 先查看程序的帮助说明，了解参数配置方法。
$ ./serial_sender -h
serial_sender description 

./serial_sender [OPTIONS]

OPTIONS:
  -h,     --help                     Print this help message and exit
  -f,     --send_frequency UINT      Set send frequency (Hz)
  -n,     --serial_port_name TEXT    Set serial port name
```

## 实验任务
1. 根据 `include/serial_comm/message.hpp` 中 `SerialMessage` 的定义，在 `serial_receiver` 的头文件和源文件中实现一个能正确解析该报文的接收程序。本项目在 CMake 中将 `serial_receiver` 定义为一个静态库（static library），`test_serial_receiver` 会链接该库并编译成用于测试的可执行文件。

2. 在确认 `serial_receiver` 功能正确后，在 `msg_publisher.cpp` 中实现一个转发节点，调用串口接收库，将 IMU 数据发布到 ROS 2 话题中。
启动该转发节点后，在新终端运行以下命令，检查话题消息的发布频率是否与 `serial_sender` 的设定一致：
```
$ ros2 topic hz /your_topic
```

### CMake 配置提示
1. 当完成 serial_receiver 接收程序后，请将 CMakeLists.txt 中的 FINISH_RECEIVER 变量置为 ON；

2. 当需要测试 serial_receiver 程序时，请将 CMakeLists.txt 中的 TEST_RECEIVER 变量置为 ON；

3. 当完成 ROS2 话题发布程序后，请将 CMakeLists.txt 中的 FINISH_PUBLISHER 变量置为 ON；

## 调试方法

使用日志打印辅助调试。项目已引入 spdlog 日志库，你也可以选择其他日志库进行输出。

由于串口通信的数据为二进制字节流，可以借助 hexdump 工具直接查看原始数据：
```
# 启动 socat 虚拟串口对
$ sudo ./start.sh
# 启动串口发送程序
$ ./serial_sender -f 100
# 打印串口原始数据
$ hexdump -C /dev/ttyUSB1 > raw.log

输出示例：

00000000  2b 01 00 00 00 f0 fa c5  19 c8 dc e5 3f c7 1c 48  |+...........?..H|
00000010  f5 62 79 d9 3f 5c 92 90  e2 79 3d e2 3f 3e 35 9c  |.by.?\...y=.?>5.|
00000020  7d a4 bd df 3f fd 1e 07  21 cb cf aa 3f 70 d5 1d  |}...?...!...?p..|
00000030  f9 65 c7 e2 3f d2 a0 87  16 66 80 ed 3f 7d d4 9b  |.e..?....f..?}..|
...
```