# nav_simulation：Gazebo场景与机器人适配

[返回主引导](../../../../README.md)

本包提供Gazebo Fortress物理环境：完整120柱场景、全向底盘、模拟雷达、真值里程计和物理接触观测。它将Gazebo数据转换为作业统一的ROS接口，并把控制指令送给底盘。

## 文件结构

| 路径 | 职责 |
| --- | --- |
| [launch/environment.launch.py](launch/environment.launch.py) | 原生Gazebo GUI、桥接、机器人TF与适配器 |
| [python/nav_simulation/adapter.py](python/nav_simulation/adapter.py) | 里程计、点云坐标变换、指令与接触适配 |
| [python/nav_simulation/world.py](python/nav_simulation/world.py)、[forest.py](python/nav_simulation/forest.py) | 生成与加载完整仿真场景 |
| [config/bridge.yaml](config/bridge.yaml) | Gazebo与ROS消息桥接配置 |
| [resource/models/](resource/models/) | 机器人模型、传感器与插件配置 |
| [resource/worlds/](resource/worlds/) | 固定柱林场景与世界模板 |
| [third_party/UPSTREAM_NOTICES.md](third_party/UPSTREAM_NOTICES.md) | 第三方来源与许可证说明 |
| [launch/kinematic.launch.py](launch/kinematic.launch.py) | 独立的简化运动学调试环境 |

## 启动真实物理环境

按主引导从宿主机执行 `./scripts/dev.sh`，在容器中运行：

```bash
ros2 launch nav_simulation environment.launch.py
```

应打开Gazebo原生窗口，显示完整柱林与机器人；机器人初始位置约为 `(29.325,8.475)`。环境默认运行，不自动发送目标。若仿真暂停，点击窗口中的播放按钮继续。

本入口不会启动学生地图、规划、控制或RViz。按主引导依次启动其他模块，保持每个模块只有一个实例。只需要无窗口运行时可使用 `gui:=false`，日常图形调试使用默认入口。

## 对外接口

| 方向 | 话题 | 类型或用途 |
| --- | --- | --- |
| 输出 | `/clock` | `rosgraph_msgs/msg/Clock`，仿真时间 |
| 输出 | `/nav/cloud` | `sensor_msgs/msg/PointCloud2`，按扫描时间变换到 `map` |
| 输出 | `/nav/odom` | `nav_msgs/msg/Odometry`，`map`到 `base_link`，速度为机体系 |
| 输出 | `/tf`、`/tf_static` | 机器人及传感器坐标关系 |
| 输出 | `/nav/collision`、`/nav/collision_status` | 接触事件与观测源健康信息 |
| 输入 | `/nav/cmd_vel` | `geometry_msgs/msg/Twist`，底盘机体系速度 |

`/nav/sim/*`用于底层桥接与适配调试；学生算法使用 `/nav/cloud` 和 `/nav/odom`。不要仅修改点云的frame名称来代替实际坐标变换。

## 调试方法

```bash
ros2 topic hz /clock
ros2 topic hz /nav/odom
ros2 topic hz /nav/cloud
ros2 topic echo /nav/collision_status --once
```

Gazebo显示完整真实场景，RViz显示导航掌握的地图。两者的障碍数量在初始状态下不同是本作业的设计。收到点云后，再检查局部地图是否能补出先验缺失的障碍。

未收到接触事件时，应结合 `/nav/collision_status` 检查观测源是否健康；不能将无消息直接解释为无碰撞。暂停Gazebo时，时钟和传感器数据也会暂停，所有下游模块应启用 `use_sim_time:=true`。

若没有窗口，先检查是否通过 `dev.sh` 进入容器、宿主机是否有图形桌面与DISPLAY、显卡透传是否配置。不要在同一ROS domain内同时启动运动学环境和Gazebo，否则会出现重复传感器和底盘接口。

## 本包作业要求

保持完整120柱场景、雷达与物理设置，使用其传感器数据完成地图任务。不得删柱、读取完整布局生成局部答案，或把完整场景预先发布为导航地图。

`kinematic.launch.py`和最终自检使用的简化环境适合快速排查软件链路；实际任务展示仍使用本包的Gazebo物理入口，按主引导保存地图、路径与运动记录。
