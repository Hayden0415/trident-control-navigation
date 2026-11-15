# nav_legacy_control：旧 PID 路径跟踪

[返回主引导](../../../../README.md)

本包将规划器的几何路径转换为全向底盘速度，并检查地图、里程计和路径是否有效。本次作业无需重写控制算法；你需要能区分地图问题、规划问题与控制器等待状态。

## 文件结构

| 文件 | 职责 |
| --- | --- |
| [src/controller_node.cpp](src/controller_node.cpp) | 路径跟踪、输入检查、速度与状态发布 |
| [include/nav_legacy_control/legacy_pid.hpp](include/nav_legacy_control/legacy_pid.hpp) | 提供的PID计算 |
| [launch/control.launch.py](launch/control.launch.py) | 独立控制入口 |
| [config/control.yaml](config/control.yaml) | 速度、加速度和输入过期时间 |
| [tests/test_legacy_pid.cpp](tests/test_legacy_pid.cpp) | PID核心测试 |

## 接口与坐标约定

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/nav/map` | `nav_msgs/msg/OccupancyGrid` | 导航图，Reliable/Transient Local |
| 输入 | `/nav/odom` | `nav_msgs/msg/Odometry` | `map`到 `base_link`，速度为机体系 |
| 输入 | `/nav/goal` | `geometry_msgs/msg/PoseStamped` | `map`中的目标位置 |
| 输入 | `/nav/planning/path` | `nav_msgs/msg/Path` | `map`中的规划路径 |
| 输出 | `/nav/cmd_vel` | `geometry_msgs/msg/Twist` | 机体系vx/vy，本题wz为0，20 Hz |
| 输出 | `/nav/control/status` | `std_msgs/msg/String` | 状态改变时发布 |

默认速度上限0.65 m/s、加速度上限1.0 m/s²、输入过期时间1秒。目标位置误差小于0.18米时停止，不验收终点朝向。控制器依靠导航图检查路径，无法凭空发现图中未标记的柱子。

## 启动与检查

先启动仿真、地图与规划，然后在容器中执行：

```bash
ros2 launch nav_legacy_control control.launch.py use_sim_time:=true
```

另开容器终端观察：

```bash
ros2 topic echo /nav/control/status
ros2 topic echo /nav/cmd_vel
```

状态只在变化时发布；订阅较晚时暂时没有状态消息，不代表节点没有运行。结合速度、路径和节点列表检查。

| 状态示例 | 含义与排查方向 |
| --- | --- |
| `waiting_for_map_and_odom` | 地图或里程计尚未收到 |
| `waiting_for_map_goal` | 尚未收到有效的map坐标目标 |
| `waiting_for_fresh_path`、`no_path` | 检查规划节点、路径消息和地图可行性 |
| `tracking` | 正在跟踪路径，结合Gazebo检查实际运动 |
| `goal_reached` | 已达到位置目标 |
| `stale_input` | 地图或里程计过期，检查更新与时钟 |
| `frame_mismatch` | 里程计坐标约定不符 |
| `path_blocked`、`start_or_goal_occupied` | 检查机器人附近的融合地图 |
| `clock_reset_or_pause` | 检查仿真暂停、时钟重置或时间跳变 |

只使用半柱全局先验时，控制器可以正常输出速度，但可能撞到先验遗漏的柱子。这是需要局部地图补足的信息。

## 本包作业要求与验证

保持控制逻辑、接口和速度限制不变；不要通过增大速度、关闭过期检查或直接给底盘持续发速度绕过地图任务。使用goal执行导航，记录跟踪和到点结果。

提供的PID核心测试可在容器工作区运行：

```bash
ctest --test-dir build/nav_legacy_control --output-on-failure
```

最终闭环和断流停车验证见主引导第6节。
