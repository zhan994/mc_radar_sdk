# mc_radar_sdk

MindCruise 4D Radar A1 的 Linux / C++11 UDP SDK。实现集中在
`include/mc_radar_sdk/radar_sdk.hpp`，共用类型位于 `common.h`，只依赖标准库和
Linux socket / eventfd，不依赖 ROS、PCL 或 Eigen。

## 调用方式

实时查看当前雷达的 target points：

```bash
cmake -S . -B build
cmake --build build --target radar_test_main
./build/radar_test_main
```

测试程序位于 `tests/test_main.cpp`，默认监听 `10.10.106.100:9999`，接收来自
`10.10.106.106` 的数据；可直接在 main 中修改这三个配置。
持续输出每帧目标点数量、设备时间戳，以及各点的 XYZ、距离、径向速度、水平角、
俯仰角、RCS、SNR 原始值和存在置信度。按 Ctrl+C 安全退出。
空帧输出 `target_points=0`；没有完整帧时每秒输出一次接收状态。

实时 3D 可视化（本机已安装 Matplotlib 和 NumPy）：

```bash
cmake -S . -B build
cmake --build build --target radar_test_main
python3 tests/view_points.py
# 或直接构建并启动窗口：
cmake --build build --target radar_viewer
```

窗口自动启动 `radar_test_main --json` 取帧，逐帧显示目标点，以径向速度着色。
X 前、Y 左、Z 上，原点为雷达；默认显示前方 20 m，可用 `--range 50` 调整。
鼠标拖动旋转，滚轮缩放，R 重置视角，空格暂停/恢复显示；关闭窗口或 Ctrl+C
会停止接收子进程。空帧清空点云，超时清除旧点并提示，状态栏显示当前帧号、
目标点数和视野之外的点数。窗口与终端程序共用 UDP 9999，应选择其中一种启动。
`--duration 5 --snapshot build/radar_3d.png` 可限时运行并保存结束时的视图。

在自己的程序中调用 SDK：

```cpp
#include <mc_radar_sdk/radar_sdk.hpp>
#include <iostream>

int main() {
  mc_radar_sdk::RadarSdk radar;
  mc_radar_sdk::RadarConfig config;
  config.port = 9999;
  // config.bind_ip = "192.168.1.10"; // 本机接收网卡地址
  // config.radar_ip = "192.168.1.20"; // 可选的雷达来源 IP
  if (!radar.Start(config)) {
    std::cerr << radar.GetStatus().last_error << '\n';
    return 1;
  }

  mc_radar_sdk::FramePtr frame;
  if (radar.WaitForFrame(frame, std::chrono::milliseconds(1000))) {
    for (const auto& point : frame->detections) {
      std::cout << point.position.x << ' ' << point.position.y << ' '
                << point.position.z << '\n';
    }
  }
  radar.Stop(); // 析构时也会自动停止。
}
```

ROS wrapper 可以在 timer 中调用非阻塞的 `GetFrames()`，或在自己的取帧线程中
调用 `WaitForFrame()`，再转换并发布 `PointCloud2`。SDK 接收线程只负责收包、
解码和组帧。`GetStatus()` 返回状态快照和错误/丢帧计数。

- `Start(config)`：启动接收，每次重新启动会清空帧队列、统计和来源选择。
  运行中再次调用返回 false；配置变更通过 Stop 后重新 Start 完成。
- `Stop()`：唤醒接收线程及等待取帧的调用，等待线程退出，再关闭文件描述符；
  可以重复调用。已完成帧仍可取出。
- `GetFrames()`：非阻塞地取走当前队列中的全部帧。
- `WaitForFrame(frame, timeout)`：取走一帧；超时、停止且队列为空或会话变化时
  返回 false，并清空输出指针。
- `ClearFrames()`：清空已完成帧队列，不影响正在组装的帧。

这些公共方法支持并发调用。多个消费者共享一个队列，每帧只交付一次；返回的
`shared_ptr<const RadarFrame>` 不可修改，可在停止、重启及 SDK 析构后继续使用。
wrapper 应先停止 SDK、结束自己的取帧调用，再销毁 SDK 对象。

## 协议与数据约定

按旧 SDK 的小端主机行为显式读取小端字段，一份 UDP 数据报对应一条报文：

```text
ID:u16 | payload_length:u16 | counter:u16 | payload | CRC:u16
0x001: 设备时间戳，至少 8 字节（前 8 字节为微秒时间戳，允许扩展字段）
0x100: 帧头，8 字节（版本、帧号、检测数量、保留字段）
0x101: 检测点，每点 24 字节
0x1FF: 帧结束（payload 未解释，兼容空 payload）
```

只有收到帧头、结束标记且检测数量与帧头相符的帧才交付。新帧头、无效报文和
超时会丢弃当前残帧，并从下一帧恢复。合法空帧正常交付。SDK 输出全部检测点，
不执行置信度/RCS过滤或固定 80 ms 跳帧。

同一次 Start 固定使用首个长度合法的时间戳或帧头报文的来源 IP 和 UDP 源端口；
其余来源会被忽略，避免多台雷达交错组帧。建议配置 `radar_ip`。
来源端口或雷达发生变化时需 Stop / Start 重新选择。

距离单位米，速度单位 m/s，角度单位度；坐标系为前左上，水平角左正、俯仰角下正。
RCS 按旧版除以 256，SNR 保留原始整数，物理量标定尚待协议确认。

帧保留设备原始微秒时间戳 `device_timestamp_us` 和帧头接收时主机
`system_clock` 纳秒时间 `host_timestamp_ns`。设备时间戳只关联下一帧，且受
`frame_timeout` 限制；`has_device_timestamp` 表示是否存在。
实机 MC.TG.45 固件发送 16 字节时间戳 payload，后 8 字节含义尚未确认，暂不解释。
设备时间的基准和同步方式未知，wrapper 不应直接把它当成 Unix/ROS 时间。
使用 ROS 仿真时间时，由 wrapper 自行决定 `header.stamp` 的赋值策略。

**协议限制：** 旧代码没有给出 CRC 算法，本实现保留 CRC 尾部长度检查，暂不校验
CRC。counter 的递增范围也未知，仅在帧中保留首尾 counter，不据此推断丢包或
纠正乱序。检测数量检查不能识别所有丢包/重复包组合；CRC、计数器及帧标识语义
需后续按正式协议补齐。

## 缓冲与状态

`max_queued_frames` 默认 32，队列满时丢弃最旧的已完成帧，并增加 `dropped_frames`。
`frame_timeout` 默认 500 ms，从帧头开始限制组帧时长；超时清理周期最多约 20 ms。
设备静默时无需重建 UDP socket，恢复发包即可继续接收。

`connection_timeout` 默认 5000 ms：`connected` 表示近期收到选定来源的合法已知
消息，`frame_valid` 表示近期组装过完整帧（包含空帧）。两者在停止时均为 false。
socket 致命错误会终止接收，并在 `last_error` 中记录原因，可重新 Start 恢复。

## 构建与验证

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

在同一个 CMake 工程中可通过 `add_subdirectory` 引入，并链接
`mc_radar_sdk::mc_radar_sdk`；它会传递头文件路径、C++11 要求和线程库。
也可直接使用头文件并在编译时添加 `-I include -pthread`。

测试通过 loopback UDP 验证字段缩放、正负值、组帧恢复、超时、队列溢出、
来源隔离、启动失败恢复、停止唤醒、并发取帧与并发启停，不需要雷达硬件。
