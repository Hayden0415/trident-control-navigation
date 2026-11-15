# nav_planning：旧 D* Lite 路径规划

[返回主引导](../../../../README.md)

本包根据导航地图、机器人位置与目标生成几何路径，供独立的控制包跟踪。旧导航的 D* Lite 实现已提供，本次作业无需重写规划器；你需要通过路径变化验证局部地图是否正确接入。

## 文件结构

| 文件 | 职责 |
| --- | --- |
| [src/legacy_dstar.cpp](src/legacy_dstar.cpp)、[include/nav_planning/legacy_dstar.hpp](include/nav_planning/legacy_dstar.hpp) | 提供的路径搜索实现 |
| [src/planner_node.cpp](src/planner_node.cpp) | ROS接口、输入检查、路径发布 |
| [launch/planning.launch.py](launch/planning.launch.py) | 独立规划入口 |
| [config/planning.yaml](config/planning.yaml) | 输入过期时间 |
| [tests/test_legacy.cpp](tests/test_legacy.cpp) | 规划核心测试 |

## 接口与运行行为

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/nav/map` | `nav_msgs/msg/OccupancyGrid` | Reliable/Transient Local，`map`坐标，0可行、100障碍、-1未知 |
| 输入 | `/nav/odom` | `nav_msgs/msg/Odometry` | 机器人在 `map` 中的位置，Best Effort兼容 |
| 输入 | `/nav/goal` | `geometry_msgs/msg/PoseStamped` | `map`中的目标位置 |
| 输出 | `/nav/planning/path` | `nav_msgs/msg/Path` | 5 Hz发布，`map`坐标，几何路径 |

规划器检查地图与里程计的新鲜度，`input_timeout`默认1秒。缺少输入、输入过期、起点被占据、图内目标被占据或无可用路径时会发布空路径。未知格不作为可通行格。该节点不发布底盘速度。

提供的适配器在目标位于地图范围外时，会尝试选取图内可达子目标；正常作业目标应位于场地内的空闲区域，不依靠这一行为完成验收。

## 启动与调试

先按主引导启动Gazebo与地图，再在容器中执行：

```bash
ros2 launch nav_planning planning.launch.py use_sim_time:=true
```

在RViz或命令行向 `/nav/goal` 发送一次目标。通过RViz的Planned path图层观察路径，检查：

1. 路径是否从机器人附近出发，并朝目标延伸。
2. 路径是否绕开 `/nav/map` 中的障碍与膨胀区。
3. 局部地图发现先验缺失的柱子后，路径是否随融合地图改变。

```bash
ros2 topic hz /nav/planning/path
ros2 topic echo /nav/planning/path --once --field header
ros2 topic info /nav/goal -v
```

空路径时先检查地图是否更新、起终点是否可行、所有模块是否使用仿真时间。仅看到路径话题存在不足以证明规划成功，需检查消息中 `poses` 是否为空。

## 本包作业要求与验证

保持规划接口和算法不变；不要修改规划器来绕过错误地图。报告中解释局部地图如何影响规划结果，提供地图与路径对应的截图或录像。

如需运行提供的核心测试，在容器工作区执行：

```bash
ctest --test-dir build/nav_planning --output-on-failure
```

该测试验证提供的规划基础设施，不能替代学生地图测试或Gazebo任务验证。
