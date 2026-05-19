# weapon_tip_detector

`weapon_tip_detector` 是一个基于 RealSense RGB-D 相机的 ROS 2 端头检测功能包。  
节点会根据当前 slot 的 ROI 区域判断是否检测到对应端头，并通过 ROS 话题发布 `0 / 1` 检测结果，供串口节点或上层逻辑使用。

本节点主要用于当前格位端头检测任务。正式比赛时，串口节点不需要读取 OpenCV 调试窗口，也不需要解析 debug image，只需要订阅检测结果话题即可。

---

## 1. 功能概述

本节点完成以下功能：

1. 订阅 RealSense 彩色图、深度图、相机内参和 depth-to-color 外参；
2. 根据当前 `current_slot_id` 选择对应 ROI；
3. 判断当前 ROI 内是否存在对应端头；
4. 在 OpenCV 窗口中显示调试信息；
5. 通过 ROS 话题发布当前检测结果：

```text
/weapon_tip_detector/current_present
```

输出数据为：

```
0：当前没有检测到目标端头
1：当前检测到目标端头
```

---

## 2\. Slot 与端头类型对应关系

当前默认 slot 映射如下：

| Slot | 端头类型 |
| :--- | :--- |
| 1 | spear |
| 2 | fist |
| 3 | palm |
| 4 | palm |
| 5 | fist |
| 6 | spear |

对应 yaml 参数：

```
slot_tip_types: ["spear", "fist", "palm", "palm", "fist", "spear"]
```

上游系统只需要告诉本节点当前正在检测哪个 slot，节点会自动使用对应的 ROI 和端头类型参数。

---

## 3\. 输入输出接口

## 3.1 输入话题

节点默认订阅 RealSense 相关话题：

```
depth_image_topic: "/camera/camera/depth/image_rect_raw"
rgb_image_topic: "/camera/camera/color/image_raw"

depth_camera_info_topic: "/camera/camera/depth/camera_info"
color_camera_info_topic: "/camera/camera/color/camera_info"
extrinsics_topic: "/camera/camera/extrinsics/depth_to_color"
```

这些输入用于将深度信息投影到彩色图像坐标系，并在当前 ROI 内完成端头检测。

---

## 3.2 当前 slot 输入

当前 slot 可以通过 launch 参数指定：

```
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3
```

也可以通过 topic 动态切换。

如果需要启用 topic 输入，在 yaml 中设置：

```
enable_slot_topic: true
current_slot_topic: "/weapon_tip_detector/current_slot_id"
```

topic 类型：

```
std_msgs/msg/UInt8
```

有效范围：

```
1 ~ 6
```

手动发布示例：

```
ros2 topic pub /weapon_tip_detector/current_slot_id std_msgs/msg/UInt8 "{data: 3}"
```

---

## 3.3 检测结果输出

节点会发布当前 slot 是否检测到对应端头：

```
/weapon_tip_detector/current_present
```

消息类型：

```
std_msgs/msg/UInt8
```

数据含义：

| data | 含义 |
| :--- | :--- |
| 0 | 当前没有检测到目标端头 |
| 1 | 当前检测到目标端头 |

示例输出：

```
data: 0
---
data: 1
---
```

正式比赛时，串口节点只需要订阅该话题，并将 `data` 字段转发给下位机即可。

---

## 4\. 与串口节点对接

本节点不直接写串口，只负责发布检测结果 topic。

串口节点需要订阅：

```
/weapon_tip_detector/current_present
```

消息类型：

```
std_msgs/msg/UInt8
```

串口节点读取：

```
msg->data
```

并按串口协议发送：

```
0：未检测到
1：检测到
```

建议串口节点只依赖该 topic，不依赖 OpenCV 调试窗口、图像颜色、框线或 debug image。

测试命令：

```
ros2 topic echo /weapon_tip_detector/current_present
```

预期输出示例：

```
data: 0
---
data: 1
---
```

查看发布频率：

```
ros2 topic hz /weapon_tip_detector/current_present
```

---

## 5\. 检测方法说明

本项目不是深度学习检测器，也不是完整点云建模，而是一个轻量化 RGB-D 检测方案。

整体思路是：

```
RealSense RGB-D 输入
        ↓
选择当前 slot ROI
        ↓
深度图投影到彩色图坐标系
        ↓
计算 ROI 内背景深度
        ↓
利用深度差 diff 提取前景
        ↓
结合暗色 mask、面积、宽高比、位置和 profile 打分
        ↓
判断当前 slot 是否存在对应端头
        ↓
发布 0 / 1 检测结果
```

---

## 6\. 深度差 diff 的含义

节点会在 ROI 内估计背景深度，并计算：

```
diff = background_depth - current_pixel_depth
```

其中：

* `background_depth`：ROI 内估计出来的背景深度；
* `current_pixel_depth`：当前像素的深度；
* `diff`：当前像素比背景更靠近相机的距离。

如果端头比背景更靠近相机，`diff` 会比较大；

如果是背景平面、墙面、阴影或纹理，`diff` 通常比较小。

因此，`diff` 可以帮助节点区分“真正凸出来的端头”和“背景上的颜色变化”。

示例：

```
背景深度：0.470 m
端头像素深度：0.430 m

diff = 0.470 - 0.430 = 0.040 m
```

如果 `diff` 达到参数中设定的前景阈值，该像素就可能被认为是前景。

---

## 7\. ROI 检测

节点不会在整张图上寻找端头，而是只在当前 slot 对应的 ROI 内检测。

ROI 在 yaml 中通过比例配置：

```
slot_roi_ratios: [
  # slot 1: spear
  0.41, 0.07, 0.18, 0.55,

  # slot 2: fist
  0.42, 0.18, 0.16, 0.38,

  # slot 3: palm
  0.37, 0.30, 0.28, 0.12,

  # slot 4: palm
  0.37, 0.30, 0.28, 0.12,

  # slot 5: fist
  0.42, 0.18, 0.16, 0.38,

  # slot 6: spear
  0.41, 0.07, 0.18, 0.55
]
```

每 4 个数表示一个 slot 的 ROI：

```
x_ratio, y_ratio, width_ratio, height_ratio
```

使用比例配置的好处是：即使相机分辨率变化，ROI 仍然能大致保持在相同画面区域。

---

## 8\. 三类端头检测策略

## 8.1 spear

`spear` 使用双 profile：

```
spear_head：偏向检测矛尖主体
spear_stem：偏向检测连接杆 / 底座区域
```

其中：

* `spear_head` 可以单独确认 spear；
* `spear_stem` 不能单独确认 spear，必须有矛尖主体暗色支撑。

这样可以减少细杆、边缘、背景竖线被误判为 spear 的情况。

---

## 8.2 fist

`fist` 使用双 profile：

```
fist_body：偏向检测拳体主体
fist_stem：偏向检测连接杆 / 下半部分
```

其中：

* `fist_body` 可以单独确认 fist；
* `fist_stem` 不能单独确认 fist，必须有拳体暗色支撑。

这样可以兼顾不同角度下 fist 的形态变化，同时减少背景细杆或阴影误检。

---

## 8.3 palm

`palm` 使用单 profile。

palm 的可见形态通常是低矮、横向、宽扁区域，不像 spear 和 fist 那样有明显的“主体 + 连接杆”结构。

因此 palm 最终采用单 profile，通过宽高比、面积、位置、暗色比例和深度前景等条件进行筛选。

---

## 9\. OpenCV 调试窗口

OpenCV 窗口只用于开发和调参阶段，方便观察：

* 当前 slot；
* 当前期望端头类型；
* ROI；
* mask；
* 候选框；
* score；
* rejected reason；
* raw / present 状态。

正式比赛时不依赖 OpenCV 窗口。

串口节点和上层逻辑只需要订阅：

```
/weapon_tip_detector/current_present
```

并读取其中的 `data` 字段。

---

## 10\. present / raw / stable

调试窗口中常见字段：

```
present: YES/NO
raw: YES/NO
stable: ON/OFF
```

含义：

| 字段 | 含义 |
| :--- | :--- |
| raw | 当前帧检测结果 |
| present | 最终输出结果 |
| stable | 是否启用历史帧稳定判断 |

如果启用 stable：

```
stable_enabled: true
stable_history_size: 5
stable_accept_count: 3
```

表示最近 5 帧中至少 3 帧检测到目标，最终 `present` 才为 YES。

发布到 `/weapon_tip_detector/current_present` 的是最终 present 结果：

```
present = YES -> data: 1
present = NO  -> data: 0
```

---

## 11\. 常见 rejected reason

调试窗口中 `best:` 后面会显示当前最佳候选的状态。

常见原因如下：

| reason | 含义 |
| :--- | :--- |
| accepted | 候选通过 |
| score_low | 综合分数不够 |
| area_small | 面积太小 |
| area_large | 面积太大 |
| width_small | 宽度比例太小 |
| height_small | 高度比例太小 |
| fill_small | 填充率太低 |
| no_depth | 候选区域没有有效深度 |
| diff_small | 深度前景差不够 |
| roi_edge | 候选太靠近 ROI 边缘 |
| dark_low | 暗色比例不足 |
| best:none | 没有形成有效候选 |

调参时应优先看 rejected reason，不建议一次大幅修改多个参数。

---

## 12\. 编译方法

进入工作空间：

```
cd ~/realsense_ws
```

编译功能包：

```
colcon build --packages-select weapon_tip_detector
source install/setup.bash
```

如果修改了 cpp，建议清理后重新编译：

```
cd ~/realsense_ws

rm -rf build/weapon_tip_detector install/weapon_tip_detector log

colcon build --packages-select weapon_tip_detector
source install/setup.bash
```

---

## 13\. 启动方法

先启动 RealSense：

```
ros2 launch realsense2_camera rs_launch.py
```

再启动端头检测节点：

```
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3
```

不同 slot 示例：

```
# slot 1: spear
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=1

# slot 2: fist
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=2

# slot 3: palm
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3
```

---

## 14\. 查看检测结果

查看当前检测输出：

```
ros2 topic echo /weapon_tip_detector/current_present
```

输出示例：

```
data: 0
---
data: 1
---
```

查看当前 slot 输入：

```
ros2 topic echo /weapon_tip_detector/current_slot_id
```

手动切换当前 slot：

```
ros2 topic pub /weapon_tip_detector/current_slot_id std_msgs/msg/UInt8 "{data: 3}"
```

---

## 15\. 文件结构

当前版本已经对原来的单文件检测节点进行了结构重构。主要结构如下：

```text
weapon_tip_detector
├── CMakeLists.txt
├── package.xml
├── README.md
├── config
│   └── current_tip_detector.yaml
├── launch
│   └── current_tip_detector.launch.py
├── include
│   └── weapon_tip_detector
│       ├── depth_projector.hpp
│       ├── detection_pipeline.hpp
│       ├── detector_profiles.hpp
│       ├── detector_types.hpp
│       └── detector_utils.hpp
└── src
    ├── current_tip_detector_node.cpp
    ├── current_tip_detector_node_refactored.cpp
    ├── depth_projector.cpp
    ├── detection_pipeline.cpp
    ├── detector_profiles.cpp
    ├── detector_utils.cpp
    └── preview_debugger.cpp
```

---

## 16\. 参数文件说明

主要参数文件：

```
config/current_tip_detector.yaml
```

其中包含：

* 当前 slot；
* slot 与端头类型映射；
* RealSense 输入话题；
* ROI 比例；
* stable 参数；
* spear profile；
* fist profile；
* palm profile；
* OpenCV 调试显示参数；
* 当前检测结果输出相关设置。

---

## 17\. 当前项目定位

当前版本完成了：

* RealSense RGB-D 输入；
* 当前 slot ROI 检测；
* depth-to-color 投影；
* foreground / dark mask；
* spear 双 profile；
* fist 双 profile；
* palm 单 profile；
* OpenCV 调试显示；
* stable 历史帧判断；
* `/weapon_tip_detector/current_present` 结果输出；
* 可与串口节点通过 ROS topic 对接。
* 代码结构重构，检测流程已从原来的单文件节点拆分为多个功能模块；

本项目当前重点不是直接控制串口，而是向外提供稳定、简单、明确的检测结果接口：

```
/weapon_tip_detector/current_present
std_msgs/msg/UInt8
0 / 1
```

串口节点、上层状态机或其他 ROS 2 节点都可以基于该接口继续集成。

需要注意的是，旧版 `current_tip_detector_node.cpp` 目前仅作为历史版本保留，后续开发和调试应以 `current_tip_detector_node_refactored.cpp` 为准。