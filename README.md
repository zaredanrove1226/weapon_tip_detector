# weapon_tip_detector

`weapon_tip_detector` 是一个基于 RealSense RGB-D 相机的 ROS 2 端头检测功能包。

节点会根据当前 `slot_id` 自动选择对应的 ROI 和检测策略，判断当前格位上是否存在对应端头，并通过 ROS 话题发布 `0 / 1` 检测结果，供串口节点、上层状态机或其他 ROS 2 节点使用。

本项目当前的重点不是直接控制串口，而是提供一个稳定、简单、明确的检测结果接口：

```text
/weapon_tip_detector/current_present
std_msgs/msg/UInt8
0 / 1
```

正式比赛或演示时，上层节点只需要订阅该话题，不需要读取 OpenCV 调试窗口，也不需要解析调试图像。

---

## 1. 功能概述

本节点完成以下功能：

1. 订阅 RealSense 彩色图、深度图、相机内参和 depth-to-color 外参；
2. 根据当前 `current_slot_id` 选择对应 slot ROI；
3. 根据 slot 类型自动选择 spear / fist / palm 检测策略；
4. 判断当前 ROI 内是否存在对应端头；
5. 在 OpenCV 窗口中显示调试信息；
6. 发布当前检测结果。

---

## 2. Slot 与端头类型

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

```yaml
slot_tip_types: ["spear", "fist", "palm", "palm", "fist", "spear"]
```

上游系统只需要告诉本节点当前检测哪个 slot，节点会自动使用对应的 ROI 和检测参数。

---

## 3. 输入输出接口

### 3.1 RealSense 输入话题

节点默认订阅 RealSense 相关话题：

```text
depth_image_topic: "/camera/camera/depth/image_rect_raw"
rgb_image_topic: "/camera/camera/color/image_raw"
depth_camera_info_topic: "/camera/camera/depth/camera_info"
color_camera_info_topic: "/camera/camera/color/camera_info"
extrinsics_topic: "/camera/camera/extrinsics/depth_to_color"
```

这些输入用于将深度信息投影到彩色图坐标系，并在当前 ROI 内完成端头检测。

---

### 3.2 当前 slot 输入

当前 slot 可以通过 launch 参数指定：

```bash
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3
```

也可以通过 topic 动态切换。

如果需要启用 topic 输入，在 yaml 中设置：

```yaml
enable_slot_topic: true
current_slot_topic: "/weapon_tip_detector/current_slot_id"
```

topic 类型：

```text
std_msgs/msg/UInt8
```

有效范围：

```text
1 ~ 6
```

手动发布示例：

```bash
ros2 topic pub /weapon_tip_detector/current_slot_id std_msgs/msg/UInt8 "{data: 3}"
```

---

### 3.3 检测结果输出

节点会发布当前 slot 是否检测到对应端头：

```text
/weapon_tip_detector/current_present
```

消息类型：

```text
std_msgs/msg/UInt8
```

数据含义：

| data | 含义 |
| :--- | :--- |
| 0 | 当前没有检测到目标端头 |
| 1 | 当前检测到目标端头 |

查看检测结果：

```bash
ros2 topic echo /weapon_tip_detector/current_present
```

查看发布频率：

```bash
ros2 topic hz /weapon_tip_detector/current_present
```

---

## 4. 与串口节点对接

本节点不直接写串口，只负责发布检测结果 topic。

串口节点只需要订阅：

```text
/weapon_tip_detector/current_present
```

消息类型：

```text
std_msgs/msg/UInt8
```

读取：

```cpp
msg->data
```

并按串口协议发送：

```text
0：未检测到
1：检测到
```

建议串口节点只依赖该 topic，不依赖 OpenCV 调试窗口、图像颜色、框线或 debug image。

---

## 5. 当前检测策略总览

本项目不是深度学习检测器，也不是完整点云建模，而是轻量化 RGB-D + ROI 检测方案。

整体流程：

```text
RealSense RGB-D 输入
        ↓
选择当前 slot ROI
        ↓
depth-to-color 投影
        ↓
根据端头类型选择检测策略
        ↓
生成 candidate mask
        ↓
连通域筛选
        ↓
根据面积、宽高、位置、深度、暗色比例等条件判断
        ↓
发布 0 / 1 检测结果
```

目前三类端头采用不同策略：

```text
spear：普通 RGB-D 检测 + 双 profile
fist：普通 RGB-D 检测 + 双 profile
palm：empty reference diff 检测
```

---

## 6. ROI 检测

节点不会在整张图上寻找端头，而是只在当前 slot 对应的 ROI 内检测。

ROI 在 yaml 中通过比例配置：

```yaml
slot_roi_ratios: [
  # slot 1: spear / 矛尖
  0.609, 0.082, 0.180, 0.700,
  # slot 2: fist / 拳
  0.570, 0.444, 0.223, 0.481,
  # slot 3: palm / 掌
  0.542, 0.611, 0.208, 0.288,
  # slot 4: palm / 掌
  0.580, 0.572, 0.180, 0.301,
  # slot 5: fist / 拳
  0.590, 0.500, 0.190, 0.360,
  # slot 6: spear / 矛尖
  0.600, 0.120, 0.170, 0.675
]
```

每 4 个数表示一个 slot 的 ROI：

```text
x_ratio, y_ratio, width_ratio, height_ratio
```

使用比例配置的好处是：即使相机分辨率变化，ROI 仍然能大致保持在相同画面区域。

当前流程中：

```text
slot_roi_ratios
  ↓
currentSlotRoi()
  ↓
display_roi
  ↓
currentDetectRoi()
  ↓
detect_roi
  ↓
检测器 evaluate(...)
```

如果：

```yaml
detect_roi_height_ratio: 1.0
```

则：

```text
detect_roi == display_roi
```

---

## 7. spear 检测策略

`spear` 使用普通 RGB-D 检测，并采用双 profile：

```text
spear_body：偏向检测矛尖主体
spear_stem：偏向检测连接杆 / 底座区域
```

其中：

* `spear_body` 可以单独确认 spear；
* `spear_stem` 不能单独确认 spear，必须有矛尖主体暗色支撑。

当前主要依赖：

* ROI；
* foreground / dark mask；
* 连通域面积；
* 宽高比例；
* 位置评分；
* 暗色比例；
* ignore mask；
* head support 约束。

`spear` 当前已经调通，主要改动集中在：

```text
slot_roi_ratios 第 1 组
spear_body_min_component_area
spear_body_min_width_ratio
spear_body_min_height_ratio
spear_body_ignore_rects
```

调试时如果出现：

```text
reason=width_small
bbox 很窄
```

通常说明检测到了细线、小碎片或 ROI 边缘干扰，不应该直接降低宽度阈值，而应优先检查 ROI 和 ignore mask。

---

## 8. fist 检测策略

`fist` 使用普通 RGB-D 检测，并采用双 profile：

```text
fist_body：偏向检测拳体主体
fist_stem：偏向检测连接杆 / 下半部分
```

其中：

* `fist_body` 可以单独确认 fist；
* `fist_stem` 不能单独确认 fist，必须有拳体暗色支撑。

当前主要依赖：

* ROI；
* foreground / dark mask；
* 连通域面积；
* 深度有效性；
* 宽高比例；
* 位置评分；
* 暗色比例；
* ignore mask；
* body support 约束。

当前 fist 调参重点：

```text
fist_body_min_component_area
fist_body_require_depth_for_candidate
fist_body_min_height_ratio
fist_stem_min_component_area
fist_stem_min_candidate_score
fist_stem_require_depth_for_candidate
```

目前已经针对小螺钉、小杆误检做了加严：

```yaml
fist_body_min_component_area: 9000
fist_body_require_depth_for_candidate: true
fist_body_min_height_ratio: 0.35
fist_stem_min_component_area: 300
fist_stem_min_candidate_score: 0.42
fist_stem_require_depth_for_candidate: true
```

如果空位误检日志显示：

```text
profile=fist_stem
reason=area_small
present=false
```

说明 stem 误检已经被面积门槛压住。

如果有 fist 时日志显示：

```text
profile=fist_body
reason=accepted
present=true
```

说明 fist 主体检测正常。

---

## 9. palm 检测策略：empty reference diff

`palm` 当前不再使用旧的 palm_body / palm_stem / palm_core / palm_expanded / density 策略。

现在 palm 统一使用 **empty reference diff**：

```text
当前图像 - 当前 slot 的空位 reference 图像 = diff
```

然后在当前 slot ROI 内，根据 RGB diff / depth diff 得到 candidate mask，再筛选连通域。

当前只在以下条件满足时走 `PalmReferenceDetector`：

```text
palm_reference_enable == true
tip_type == "palm"
current_slot_id == 3 或 4
```

其他 slot 仍然走普通 detection pipeline。

成功日志通常类似：

```text
profile=palm_reference_diff
cand_mode=reference_diff
reason=accepted_reference_diff
```

如果 palm 日志出现普通旧 profile，例如：

```text
profile=palm
profile=palm_expanded_reject
cand_mode=foreground_and_dark
```

说明没有进入 reference diff，需要检查：

* `palm_reference_enable`；
* current slot 是否为 3 / 4；
* reference 图片是否加载成功；
* 文件路径是否正确。

---

## 10. palm reference 图片

slot3 和 slot4 的 palm 必须分别采集 empty reference，因为相机视角、ROI 和背景都不同。

当前 reference 文件：

```text
config/reference/slot3_empty_Color.png
config/reference/slot3_empty_Depth.png
config/reference/slot4_empty_Color.png
config/reference/slot4_empty_Depth.png
```

对应 yaml：

```yaml
palm_reference_slot3_empty_rgb_path: "config/reference/slot3_empty_Color.png"
palm_reference_slot3_empty_depth_path: "config/reference/slot3_empty_Depth.png"
palm_reference_slot4_empty_rgb_path: "config/reference/slot4_empty_Color.png"
palm_reference_slot4_empty_depth_path: "config/reference/slot4_empty_Depth.png"
```

启动时应看到：

```text
Palm reference slot3 load: ok
Palm reference slot4 load: ok
```

如果看到：

```text
Palm reference slot3 load: failed
Palm reference images not loaded for slot 3. Using normal detection.
```

说明 slot3 reference 图片没有被正确加载，节点会退回普通检测逻辑。

---

## 11. palm reference 参数

当前 palm slot3 参数示例：

```yaml
palm_reference_slot3_empty_rgb_path: "config/reference/slot3_empty_Color.png"
palm_reference_slot3_empty_depth_path: "config/reference/slot3_empty_Depth.png"
palm_reference_slot3_dark_gray_threshold: 90
palm_reference_slot3_rgb_diff_threshold: 35
palm_reference_slot3_depth_delta: 0.04
palm_reference_slot3_allow_depth_hole: true
palm_reference_slot3_min_area: 3000
palm_reference_slot3_min_width: 54
palm_reference_slot3_min_height: 35
palm_reference_slot3_min_aspect_ratio: 0.35
palm_reference_slot3_min_fill_ratio: 0.05
```

当前 palm slot4 参数示例：

```yaml
palm_reference_slot4_empty_rgb_path: "config/reference/slot4_empty_Color.png"
palm_reference_slot4_empty_depth_path: "config/reference/slot4_empty_Depth.png"
palm_reference_slot4_dark_gray_threshold: 90
palm_reference_slot4_rgb_diff_threshold: 35
palm_reference_slot4_depth_delta: 0.04
palm_reference_slot4_allow_depth_hole: true
palm_reference_slot4_min_area: 3000
palm_reference_slot4_min_width: 80
palm_reference_slot4_min_height: 40
palm_reference_slot4_min_aspect_ratio: 0.45
palm_reference_slot4_min_fill_ratio: 0.08
```

调参原则：

* 小螺钉、小碎片误检：优先提高 `min_area`、`min_width`、`min_height`；
* 端头正常放置检测不到：先看日志中的 `area`、`bbox`、`pass`、`reason`，不要盲目降低全部阈值；
* slot3 和 slot4 分开调，不共用 reference，也不共用 ROI；
* 不要重新引入旧 palm 策略。

---

## 12. 深度差 diff 的含义

普通检测策略中，节点会在 ROI 内估计背景深度，并计算：

```text
diff = background_depth - current_pixel_depth
```

其中：

* `background_depth`：ROI 内估计出来的背景深度；
* `current_pixel_depth`：当前像素的深度；
* `diff`：当前像素比背景更靠近相机的距离。

如果端头比背景更靠近相机，`diff` 会比较大。

如果是背景平面、墙面、阴影或纹理，`diff` 通常比较小。

示例：

```text
背景深度：0.470 m
端头像素深度：0.430 m
diff = 0.470 - 0.430 = 0.040 m
```

在 reference diff 检测中，日志里的 `diff` 不一定是主要判断依据。palm 更主要看 reference diff 后形成的 candidate mask 和连通域筛选结果。

---

## 13. OpenCV 调试窗口

OpenCV 窗口只用于开发和调参阶段，方便观察：

* 当前 slot；
* 当前期望端头类型；
* ROI；
* mask；
* 候选框；
* score；
* rejected reason；
* raw / present 状态。

正式比赛时不依赖 OpenCV 调试窗口。

串口节点和上层逻辑只需要订阅：

```text
/weapon_tip_detector/current_present
```

并读取其中的 `data` 字段。

---

## 14. present / raw / stable

调试窗口中常见字段：

```text
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

```yaml
stable_enabled: true
stable_history_size: 5
stable_accept_count: 3
```

表示最近 5 帧中至少 3 帧检测到目标，最终 `present` 才为 YES。

发布到 `/weapon_tip_detector/current_present` 的是最终 present 结果：

```text
present = YES -> data: 1
present = NO  -> data: 0
```

调参阶段建议：

```yaml
stable_enabled: false
```

演示阶段如果画面稳定，可以打开：

```yaml
stable_enabled: true
```

---

## 15. 常见 rejected reason

调试窗口中 `reason` 会显示当前最佳候选的状态。

常见原因如下：

| reason | 含义 |
| :--- | :--- |
| accepted | 普通检测候选通过 |
| accepted_reference_diff | palm reference diff 候选通过 |
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
| No candidate passes thresholds | 没有候选通过阈值 |
| best:none | 没有形成有效候选 |

调参时应优先看：

```text
profile
cand_mode
present / raw
reason
area
bbox
pos
dark_ratio
depth_count
```

不要一次大幅修改多个参数。

---

## 16. 编译方法

进入工作空间：

```bash
cd ~/realsense_ws
```

编译功能包：

```bash
colcon build --packages-select weapon_tip_detector --symlink-install
source install/setup.bash
```

如果修改了 cpp / hpp，建议清理后重新编译：

```bash
cd ~/realsense_ws
rm -rf build/weapon_tip_detector install/weapon_tip_detector log
colcon build --packages-select weapon_tip_detector --symlink-install
source install/setup.bash
```

如果只是修改 yaml，一般重新 source 后重新 launch 即可。

---

## 17. 启动方法

当前 launch 已经可以把 RealSense launch 合并进来，因此通常只需要启动一个 launch：

```bash
cd ~/realsense_ws
source install/setup.bash
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3 target_distance:=0.50
```

不同 slot 示例：

```bash
# slot 1: spear
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=1 target_distance:=0.50

# slot 2: fist
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=2 target_distance:=0.50

# slot 3: palm
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=3 target_distance:=0.50

# slot 4: palm
ros2 launch weapon_tip_detector current_tip_detector.launch.py current_slot_id:=4 target_distance:=0.50
```

如果 RealSense 已经由其他 launch 启动，应避免重复启动相机。

---

## 18. 查看检测结果

查看当前检测输出：

```bash
ros2 topic echo /weapon_tip_detector/current_present
```

输出示例：

```text
data: 0
---
data: 1
---
```

查看发布频率：

```bash
ros2 topic hz /weapon_tip_detector/current_present
```

查看当前 slot 输入：

```bash
ros2 topic echo /weapon_tip_detector/current_slot_id
```

手动切换当前 slot：

```bash
ros2 topic pub /weapon_tip_detector/current_slot_id std_msgs/msg/UInt8 "{data: 3}"
```

---

## 19. 文件结构

当前版本已经对原来的单文件检测节点进行了结构重构。主要结构如下：

```text
weapon_tip_detector
├── CMakeLists.txt
├── package.xml
├── README.md
├── config
│   ├── current_tip_detector.yaml
│   └── reference
│       ├── slot3_empty_Color.png
│       ├── slot3_empty_Depth.png
│       ├── slot4_empty_Color.png
│       └── slot4_empty_Depth.png
├── launch
│   └── current_tip_detector.launch.py
├── include
│   └── weapon_tip_detector
│       ├── depth_projector.hpp
│       ├── detection_pipeline.hpp
│       ├── detector_profiles.hpp
│       ├── detector_types.hpp
│       ├── detector_utils.hpp
│       ├── palm_reference_detector.hpp
│       └── preview_debugger.hpp
└── src
    ├── current_tip_detector_node.cpp
    ├── current_tip_detector_node_refactored.cpp
    ├── depth_projector.cpp
    ├── detection_pipeline.cpp
    ├── detector_profiles.cpp
    ├── detector_utils.cpp
    ├── palm_reference_detector.cpp
    └── preview_debugger.cpp
```

说明：

* `current_tip_detector_node.cpp`：旧版历史文件，保留作参考；
* `current_tip_detector_node_refactored.cpp`：当前实际使用的主节点；
* `detection_pipeline.*`：普通 spear / fist 检测流程；
* `palm_reference_detector.*`：palm empty reference diff 检测流程；
* `detector_profiles.*`：端头 profile 默认参数；
* `detector_types.*`：检测结构体和数据类型；
* `preview_debugger.*`：OpenCV 调试窗口绘制；
* `depth_projector.*`：深度到彩色图坐标投影相关逻辑。

---

## 20. 当前项目状态

当前版本完成了：

* RealSense RGB-D 输入；
* 当前 slot ROI 检测；
* depth-to-color 投影；
* spear 双 profile；
* fist 双 profile；
* palm empty reference diff；
* slot3 / slot4 palm 独立 reference；
* OpenCV 调试显示；
* stable 历史帧判断；
* `/weapon_tip_detector/current_present` 结果输出；
* 与串口节点通过 ROS topic 对接；
* 代码结构重构，检测流程已从原来的单文件节点拆分为多个功能模块。

当前调试结论：

* `spear`：已通过 ROI 和 profile 参数调通；
* `fist`：已通过 body / stem 参数加严减少误检；
* `palm`：已切换到 reference diff 策略，slot3 和 slot4 分别使用独立 empty reference；
* 旧 palm_body / palm_stem / palm_core / palm_expanded / density 策略已经废弃，不应再恢复。

后续开发和调试应以：

```text
current_tip_detector_node_refactored.cpp
```

为准。