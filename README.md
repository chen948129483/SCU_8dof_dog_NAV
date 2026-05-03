# SCURC_Nav_Sim（estimations）快速使用说明

本 README 面向“拿到代码后直接跑起来”。内容基于当前仓库里的实际代码与启动链路整理，覆盖两条主流程：

- 点到点导航（RViz 下发单目标）
- S 型绕杆（`NavigateThroughPoses`，手动触发）

---

## 1. 当前工程核心能力

- 导航主链路：`leg_bringup`（重定位 + Nav2）
- 运动控制：`dog_discrete_controller`（离散步态控制器插件）
- 下位机串口桥：`usart_pkg`（`/dog_step_cmd` → 串口帧）
- 绕杆任务：`nav_pose`（新包，手动触发 through-poses，暂时未经过验证，但是主要目标是按照指定的点行走从而实现绕杆功能）

关键入口文件：



- 启动总入口：`src/usart_pkg/launch/bringup_then_usart.launch.py`
或者是，
先启动导航：`robot_functionality/leg_bringup/launch/bringup_in_real.launch.py`
然后启动串口发送指令的包：src/usart_pkg

- 实现绕杆任务目前写的脚本：`src/nav_pose/nav_pose/slalom_through_poses.py`

对导航任务进行微调主要的的文件
- 导航参数：`robot_functionality/leg_bringup/params/nav2_params.yaml`


---

## 2. 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- 已可用的 Nav2 相关依赖（`nav2_bringup`、`nav2_simple_commander` 等）
- 串口库 `LibSerial`（`usart_pkg` 依赖）

> 说明：本仓库包含多个包，如需安装，可以先

---

## 3. 推荐构建方式（最小可用集）

在工作区根目录执行：

```bash
cd /home/cd/Nav_dog/pre/SCURC_Nav_Sim/estimations
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  usart_pkg \
  dog_discrete_controller \
  costmap_intensity \
  behavior_ext_plugins \
  velocity_smoother_ext \
  leg_bringup \
  nav_pose

source install/setup.bash
```

如果你已经编过，只改了 `nav_pose`，可单独：

```bash
colcon build --packages-select nav_pose
source install/setup.bash
```

---

## 4. 一键启动“点到点导航 + 串口”

### 4.1 启动

```bash
ros2 launch usart_pkg bringup_then_usart.launch.py
```

可选参数：

- `use_sim_time`（默认 `false`）
- `usart_delay_sec`（默认 `25.0`，控制 `usart_node` 延迟启动）

例如：

```bash
ros2 launch usart_pkg bringup_then_usart.launch.py use_sim_time:=false usart_delay_sec:=30.0
```

### 4.2 在 RViz 下发目标

- 打开 RViz 后，使用 `2D Goal Pose` 在地图上点一个目标点。
- 机器人会走 Nav2 全局规划 + 本地离散控制到目标。

---

## 5. S 型绕杆（手动触发）

`nav_pose` 不会自动跟随 launch 启动，需你手动执行：

```bash
ros2 run nav_pose slalom_through_poses
```

脚本特点：

- 固定拓扑骨架 + 动态偏置参数
- 默认第一根杆从右侧开始（左右交替）
- 自动根据轨迹切线计算航向
- 通过 `goThroughPoses()` 连续执行多点

### 5.1 常用参数（`--ros-args -p`）

- `frame_id`：坐标系，默认 `map`
- `pole_points`：杆中心点序列（分号分隔），默认 `0.0,0.4;-0.3,1.7;1.0,1.4;2.0,1.4`
- `start_anchor`：进入段锚点，默认 `-0.5,0.2`
- `end_anchor`：终点锚点，默认 `2.4,1.4`
- `offset`：绕杆横向偏置，默认 `0.40`
- `blend_distance`：entry/apex/exit 平滑距离，默认 `0.18`
- `start_from_right`：是否第一根从右侧开始，默认 `true`

示例：

```bash
ros2 run nav_pose slalom_through_poses --ros-args \
  -p start_from_right:=true \
  -p offset:=0.38 \
  -p blend_distance:=0.20 \
  -p pole_points:="0.0,0.4;-0.3,1.7;1.0,1.4;2.0,1.4"
```

---

## 6. 已对绕杆做的保守参数调整

文件：`robot_functionality/leg_bringup/params/nav2_params.yaml`

为“慢但稳”绕杆，已设置：

- `controller_server.general_goal_checker.xy_goal_tolerance: 0.22`
- `local_costmap.local_inflation_layer.inflation_radius: 0.16`
- `local_costmap.local_inflation_layer.cost_scaling_factor: 3.2`
- `global_costmap.global_inflation_layer.inflation_radius: 0.16`
- `global_costmap.global_inflation_layer.cost_scaling_factor: 4.2`

> 若出现“杆间通道堵死”，优先继续小幅降低 `inflation_radius`（每次 0.01~0.02）；若贴杆太近，再回调增大。

---

## 7. 话题与协议速查

### 7.1 控制相关

- 控制器发布离散动作：`/dog_step_cmd`（`usart_pkg/msg/Action`）
- 串口反馈执行状态：`/dog_status/is_stepping`（`std_msgs/Bool`）

`Action.msg` 动作定义：

- `1` 站立
- `2` 前进
- `9` 后退
- `4` 左转
- `5` 右转

### 7.2 串口默认参数

- 设备：`/dev/ttyACM0`
- 波特率：`115200`
- 帧格式：`55 AA action speed checksum 0D 0A`

---

## 8. 常见问题排查

### 8.1 找不到路径 / 杆间无法通过

- 先看 `nav2_params.yaml` 的 `inflation_radius` 是否过大。
- 再检查障碍输入话题（当前局部层使用 `/terrain_map`）。

### 8.2 机器人在中间点“扭头打摆”

- 适当增大 `xy_goal_tolerance`（已设为 `0.22`）。
- 适当增大 `blend_distance`，让 through-poses 转弯更顺。

### 8.3 串口打不开

- 检查设备是否存在：`ls /dev/ttyACM*`
- 检查权限（必要时加入 `dialout` 组并重新登录）

### 8.4 新包命令找不到

- 确认执行过：
  - `colcon build --packages-select nav_pose`
  - `source install/setup.bash`

---

## 9. 建议运行顺序（实操）

1. `source /opt/ros/humble/setup.bash`
2. `source install/setup.bash`
3. 启动主系统：`ros2 launch usart_pkg bringup_then_usart.launch.py`
4. 验证点到点：RViz 点 `2D Goal Pose`
5. 手动触发绕杆：`ros2 run nav_pose slalom_through_poses`

---

## 10. 后续建议

如果你后续希望“参数可复用”，建议新增一个 `nav_pose` 的专用 launch（只做参数封装，不自动启动），把不同赛道的 `pole_points/offset/blend_distance` 做成 profile 文件。
