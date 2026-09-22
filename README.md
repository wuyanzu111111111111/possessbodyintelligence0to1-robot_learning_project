# M3：ROS 2 视觉 Topic 闭环

本项目实现一个基于 ROS 2 Humble 的合成视觉目标跟随闭环，用固定生成的图像验证感知、控制和安全停车逻辑。

## 功能

系统包含三个独立节点：

1. `image_publisher_node`
   - 生成固定的 320×240 BGR 合成图像。
   - 发布到 `/camera/image_raw`。
   - 支持 `left`、`center`、`right`、`near`、`no_target` 和 `silence` 六种场景。

2. `image_detector_node`
   - 订阅 `/camera/image_raw`。
   - 使用 HSV 阈值、轮廓面积和轮廓矩检测黄色目标。
   - 发布 `target_interfaces/msg/TargetInfo` 到 `/target_info`。

3. `target_follower_node`
   - 订阅 `/target_info`。
   - 根据横向误差和目标面积代理值发布 `/cmd_vel`。
   - 目标未检出、目标过近或消息超时时发布零速度。

## 数据流

```text
image_publisher_node
    │
    │ /camera/image_raw
    │ sensor_msgs/msg/Image
    ▼
image_detector_node
    │
    │ /target_info
    │ target_interfaces/msg/TargetInfo
    ▼
target_follower_node
    │
    │ /cmd_vel
    │ geometry_msgs/msg/Twist
    ▼
机器人速度接口
```

## TargetInfo字段
```
builtin_interfaces/Time stamp
bool detected
float32 lateral
float32 distance
```
字段含义：
- detected：当前图像中是否检测到目标。
- lateral：归一化横向误差，负数表示左侧，正数表示右侧。
- distance：目标轮廓面积占图像面积的比例，是距离代理值，不是以米为单位的真实距离。

## 构建
```
cd ~/robot_project/M3_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select target_interfaces target_perception \
  --symlink-install

source install/setup.bash
```
## 启动完整链路
```
ros2 launch target_perception m3_pipeline.launch.py
```
查看可配置参数：
```
ros2 launch \
  target_perception \
  m3_pipeline.launch.py \
  --show-args
```
指定启动场景：
```
ros2 launch \
  target_perception \
  m3_pipeline.launch.py \
  scenario:=left
```
检查节点和话题
```
ros2 node list
ros2 topic list -t
```
预期节点：
```
/image_publisher_node
/image_detector_node
/target_follower_node
```
关键话题：
```
/camera/image_raw  sensor_msgs/msg/Image
/target_info       target_interfaces/msg/TargetInfo
/cmd_vel           geometry_msgs/msg/Twist
```
可重复场景测试
场景通过 ROS 参数选择，不使用随机数，也不依赖 GUI 拖动。
通用测试命令：
```
ros2 param set /image_publisher_node scenario <场景名称>
sleep 1

ros2 topic echo --once /target_info
ros2 topic echo --once /cmd_vel
```
验收结果：
场景	TargetInfo关键结果	cmd_vel关键结果
```
left	detected=true，lateral=-0.5	linear.x=0.2，angular.z=0.4
center	detected=true，lateral=0	linear.x=0.2，angular.z=0
right	detected=true，lateral=0.5	linear.x=0.2，angular.z=-0.4
near	detected=true，distance≈0.1214	线速度和角速度均为0
no_target	detected=false	线速度和角速度均为0
silence	不再产生新的TargetInfo	超过1秒后因看门狗超时停车
```

消息断流测试：
```
ros2 param set /image_publisher_node scenario silence
sleep 1.5
ros2 topic echo --once /cmd_vel
```
恢复正常输入：
```
ros2 param set /image_publisher_node scenario center
```
no_target 与 silence 含义不同：
- no_target 仍然发布图像，并产生新的 detected=false 消息。
- silence 不再发布图像，导致 /target_info 断流并触发看门狗。
## 参数实验
普通目标轮廓面积约为958像素。
提高最小轮廓面积：
```
ros2 param set /image_detector_node min_contour_area 1200.0
```
由于 958 < 1200，中心目标变为：
detected=false
cmd_vel全部为0
恢复默认值：
```
ros2 param set /image_detector_node min_contour_area 500.0
```
目标重新被检测，中心场景恢复：
```
detected=true
linear.x=0.2
angular.z=0
```
测试
```
colcon test --packages-select target_perception
colcon test-result --verbose
```
当前结果：
3 tests, 0 errors, 0 failures, 1 skipped
## 已知边界
- 当前输入为合成黄色目标，不是真实摄像头。
- distance 是轮廓面积比例，不是经过标定的真实距离。
- launch 启动初期，跟随节点可能先于感知链路就绪，因此会先安全停车并输出一次尚未收到消息的警告。
- 当前 launch 默认不会自动重启异常退出的节点。
## 问题复盘
虚拟机中出现错误的消息超时
症状：正常发布时，跟随节点反复输出超时和恢复。
根因：看门狗使用可能因虚拟机校时而跳变的系统/ROS时间计算经过时间。
修复：定时器和看门狗改用 ClockType.STEADY_TIME；消息时间戳继续使用ROS时间。
构建后误以为launch文件没有安装
症状：使用 find -type f 时没有看到launch文件。
根因：项目使用 --symlink-install，launch文件在安装空间中是符号链接，不属于普通文件。
修复：同时检查普通文件和符号链接：
```
find install/target_perception/share/target_perception \
  -maxdepth 2 \
  \( -type f -o -type l \) \
  -print
  ```