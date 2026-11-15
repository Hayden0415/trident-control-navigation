# nav_obstacle_map：局部障碍地图作业

[返回主引导](../../../../README.md)

本包是本次作业需要实现的模块。它接收当前雷达点云和机器人里程计，建立随机器人移动的二维局部障碍图，再与提供的半柱全局先验融合，供规划和控制使用。

## 文件结构与修改范围

| 文件 | 用途 | 作业要求 |
| --- | --- | --- |
| [student_map.py](python/nav_obstacle_map/student_map.py) | `GridSpec`和两个算法函数 | 只补全 `project_cloud`、`inflate_grid`，保留接口 |
| [mapper_node.py](python/nav_obstacle_map/mapper_node.py) | 点云接收、滚动窗口、时间戳与ROS发布 | 框架提供，阅读调用流程，无需修改 |
| [global_prior.py](python/nav_obstacle_map/global_prior.py) | 先验栅格与局部障碍叠加 | 框架提供，无需修改 |
| [prior_node.py](python/nav_obstacle_map/prior_node.py) | 发布先验与融合导航图 | 框架提供，无需修改 |
| [mapping.launch.py](launch/mapping.launch.py) | 地图模块启动入口 | 用于自由调试 |
| [mapping.yaml](config/mapping.yaml) | 地图参数 | 理解参数，测试时保持可追溯 |
| [tests/](tests/) | 算法与先验测试 | 运行现有测试，报告自选边界案例 |
| [resource/](resource/) | 提供的60柱先验 | 不补全缺失柱子，不替换场景数据 |

## 输入与输出

| 方向 | 话题 | 类型与约定 |
| --- | --- | --- |
| 输入 | `/nav/cloud` | `PointCloud2`，当前雷达扫描，坐标已转换为 `map` |
| 输入 | `/nav/odom` | `Odometry`，用于计算局部窗口的位置 |
| 输出 | `/nav/map/raw` | `OccupancyGrid`，学生函数生成的未膨胀投影 |
| 输出 | `/nav/map/local` | `OccupancyGrid`，启用先验时的局部膨胀图 |
| 输出 | `/nav/map/prior` | `OccupancyGrid`，提供的全局先验 |
| 输出 | `/nav/map` | `OccupancyGrid`，规划和控制实际使用的导航图 |

话题类型使用 `sensor_msgs/msg/PointCloud2`、`nav_msgs/msg/Odometry` 和 `nav_msgs/msg/OccupancyGrid`。地图均为 `map` 坐标，采用Reliable/Transient Local。关闭全局先验时，学生膨胀图直接发布到 `/nav/map`，不会额外发布 `/nav/map/local`。

局部窗口的位置、消息时间戳和坐标适配由封装处理。两个学生函数处理NumPy数组，不需要自行编写ROS订阅或发布。

## 必做任务与算法约定


保留函数签名、`GridSpec`与返回约定，不读取完整世界布局，不使用固定障碍坐标或预生成答案。

### A. 当前雷达点云投影到二维局部栅格

输入 `points` 是形状 `(N,3)` 的 NumPy 数组，列依次为 `map` 坐标下的 `x,y,z`，单位米。每次输入是当前雷达扫描，不是完整世界地图。`spec` 描述由框架按机器人位置滚动的局部窗口。

返回形状 `(height,width)`、dtype为 `uint8` 的数组：无障碍命中为0，有障碍命中为100。

1. 忽略包含 NaN 或 Inf 的点。
2. 只保留 `z_min <= z <= z_max` 的点，包含端点。
3. 以窗口左下角为原点，使用 `floor((coordinate-origin)/resolution)` 求格号。
4. 左、下边界包含，右、上边界不包含；越界点忽略。
5. 同一 XY 格存在任意有效点即可标为100，重复点不改变结果。
6. 数组索引为 `[iy,ix]`，发布时按行展开，即 `ix+iy*width`。
7. 每帧从零重建，不保留上一帧的占据状态。障碍离开当前扫描后不应残留。

这里的0只表示当前扫描没有命中障碍，并不保证未观测区域真实可通行；这是本题的简化二值模型。

### B. 二维圆盘膨胀

以原始占据格为种子，把格子中心距离 `<= inflation_radius + 1e-9` 米的格子标为100。要求使用欧氏圆盘，地图边界处截断，返回独立副本，不能修改输入。半径为0时内容不变。不能把新增占据格继续作为种子，造成连锁膨胀。

自由调试默认使用0.1米分辨率、100×80格的局部窗口、高度范围 `[0.35,2.5]` 米、0.4米膨胀半径。参数可在地图包配置与 launch 参数中调整；全局与局部地图必须使用同一分辨率并对齐栅格。

全局先验是300×180格、原点 `(0,-1)` 的静态图，只包含60根柱子。提供的融合节点保留先验障碍，再叠加当前局部障碍；不二次膨胀、不累积局部历史。首次收到有效局部图前，导航图以5 Hz发布全局先验，支持未完成作业时观察仅靠全局地图行驶的效果。收到有效局部图后，融合图保留局部图的时间戳；此后局部输入过期时控制器停车，不自动退回仅使用先验。

## 启动与编译

先按[主引导](../../../../README.md)启动Gazebo，再在容器中执行：

```bash
ros2 launch nav_obstacle_map mapping.launch.py use_sim_time:=true use_global_prior:=true
```

自由调试入口将膨胀半径设为0.4米，覆盖 `mapping.yaml` 中0.3米的默认值；`GridSpec`自身的默认值也不代表Gazebo启动配置。需要调整时显式传入参数，并在报告中记录：

```bash
ros2 launch nav_obstacle_map mapping.launch.py use_sim_time:=true use_global_prior:=true inflation_radius:=0.4
```

采用源码挂载开发时，先停止地图launch，再在容器工作区重新构建并重启：

```bash
colcon build --packages-select nav_obstacle_map
source /opt/nav_ws/install/setup.bash
ros2 launch nav_obstacle_map mapping.launch.py use_sim_time:=true use_global_prior:=true
```

## 分步验证

1. 先在容器工作区运行算法测试：

   ```bash
   PYTHONPATH=src/nav/obstacle-map/nav_obstacle_map/python python3 -m pytest -q src/nav/obstacle-map/nav_obstacle_map/tests/test_student_map.py
   PYTHONPATH=src/nav/obstacle-map/nav_obstacle_map/python python3 -m pytest -q src/nav/obstacle-map/nav_obstacle_map/tests/test_global_prior.py
   ```

   测试命令通过 `PYTHONPATH` 读取当前源码，不依赖上一次编译安装的版本。模板中的两个函数未实现时，学生算法测试失败是预期状态；不能修改测试使模板通过。

2. 在RViz检查 `/nav/map/raw`，确认地面过滤、XY位置和边界处理正确，再查看 `/nav/map/local` 的圆盘膨胀与滚动更新。
3. 检查 `/nav/map` 是否保留先验并加入局部障碍。初始仅先验也可以行驶，仍需要局部地图消除对缺失柱子的盲区。
4. 停止手动运行的各个launch，再运行 `python3 scripts/check_live_map.py`，检查连续点云、窗口移动、时间戳和乱序输入。最终闭环自检见主引导第6节。

常见错误包括：直接整数截断代替floor、交换 `[iy,ix]`、误包含上/右边界、把地面投影为障碍、连锁膨胀、保留上一帧障碍、只改frame名称而不转换坐标。

## 完成标准与提交

两个函数通过测试；局部地图根据当前扫描实时变化；Gazebo任务中能观察到新增障碍改变融合图和规划结果。提交代码、方法与复杂度说明、测试输出、自选边界案例及包含地图和路径的任务记录。全作业评分与提交清单见[主引导](../../../../README.md)。
