# nav_bringup：组合启动与RViz调试

[返回主引导](../../README.md)

本包组织已有模块的启动方式并提供RViz配置。本次作业无需在此实现算法。日常开发优先按主引导逐个启动模块，便于查看各终端输出和单独重启。

## 文件与入口

| 文件 | 职责与使用时机 |
| --- | --- |
| [launch/rviz.launch.py](launch/rviz.launch.py) | 打开RViz2，默认使用仿真时间，加载作业显示配置 |
| [config/course.rviz](config/course.rviz) | `map`固定坐标、地图/点云/路径/位姿图层与目标工具 |
| [launch/navigation.launch.py](launch/navigation.launch.py) | 同时启动规划与控制，不包含仿真或地图 |
| [launch/course.launch.py](launch/course.launch.py) | 最终自动墙体运动学夹具入口，不是Gazebo启动入口 |

## 日常使用RViz

先按主引导启动Gazebo、地图、规划与控制，在另一个容器终端执行：

```bash
ros2 launch nav_bringup rviz.launch.py
```

确认Fixed Frame为 `map`，使用以下图层对照数据：

| 图层 | 话题 | 观察内容 |
| --- | --- | --- |
| Global prior | `/nav/map/prior` | 已知的60柱全局地图 |
| Navigation map | `/nav/map` | 规划和控制正在使用的地图 |
| Local inflated map | `/nav/map/local` | 学生局部膨胀结果，默认关闭 |
| LiDAR points | `/nav/cloud` | 当前三维扫描 |
| Planned path | `/nav/planning/path` | 规划输出 |
| Robot pose | `/nav/odom` | 当前机器人位姿 |
| TF | `/tf`、`/tf_static` | 坐标关系，默认关闭 |

在Displays中勾选需要的图层；对照先验、局部和融合图时可分别切换显示，避免相互遮挡。原始投影可新增Map显示，将话题设为 `/nav/map/raw`。地图使用Reliable/Transient Local，点云使用Best Effort/Volatile。

## 发送目标并观察任务

选择工具栏 **2D Goal Pose（SetGoal）**，确认Topic为 `/nav/goal`，在空闲区域按住左键拖动后松开。目标使用 `map` 坐标，消息类型为 `geometry_msgs/msg/PoseStamped`。本作业验收目标位置，不验收终点朝向。

也可在容器中执行：

```bash
ros2 topic pub --once -w 1 /nav/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.475, y: 4.375, z: 0.0}, orientation: {w: 1.0}}}'
```

发送一次目标后观察路径与运动；需要更换任务时再发送新目标。不要持续高频重复发布同一个goal，这会反复重置任务状态。

初始仅先验时就能发goal行驶，可能撞到地图缺失的柱子。完成作业后，应能观察到当前扫描产生的局部障碍进入融合图，并影响路径。

## 可选组合入口与最终自检

已有仿真和地图时，可以用下列命令同时启动规划与控制：

```bash
ros2 launch nav_bringup navigation.launch.py use_sim_time:=true
```

使用它之前先停止单独的planning/control launch，避免重复节点。RViz仍需单独启动。

`course.launch.py`用于自动运动学夹具，默认会启动自己的仿真数据、地图、规划与控制。最终自检脚本负责相应测试流程；运行前停止手动模块，步骤见主引导第6节。不要用它替代Gazebo入口。

## 本包作业要求

使用RViz验证原始投影、局部膨胀和融合图，记录地图、路径、机器人位姿与任务结果。可以调整图层显示用于调试，但保持话题和坐标接口不变，不用显示配置掩盖数据错误。无需编写自定义UI。

## 自动录像与打包

最终提交在宿主机运行 `python3 scripts/submit.py --student-id 编号 --report report.md`。脚本会重新构建镜像、运行自检，再独立启动Gazebo与RViz、排列窗口、发送固定目标并录制最终视频，最后压缩源码与证据。详细依赖、目录与失败处理见[主引导第6节](../../README.md)。不要与手动launch同时进行任务演示。
