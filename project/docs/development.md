# 开发与接入

[返回主引导](../README.md)

日常启动步骤见主引导第3节，算法要求见[地图包README](../src/nav/obstacle-map/nav_obstacle_map/README.md)。需要在宿主机修改代码、在同一个容器中反复编译时，使用以下模式。

## 挂载源码开发

宿主机在作业根目录执行：

```bash
TRIDENT_DEV=1 ./scripts/dev.sh
```

该模式挂载当前仓库的src到/opt/nav_ws/src，图形和ROS环境与普通入口相同。宿主机修改学生函数后，先停止地图模块，再在容器中构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav_obstacle_map
source /opt/nav_ws/install/setup.bash
ros2 launch nav_obstacle_map mapping.launch.py use_sim_time:=true
```

正常构建会将Python文件安装到install目录，编辑后需重建并重启对应节点。只改地图包时，无需重建规划或控制。其他终端通过 `docker exec -it trident-course /course-entrypoint.sh bash` 进入，加载同一install。容器退出后其中build/install会被删除，宿主机src中的修改保留；最后提交前仍应重新构建镜像验收。

## 接入统一框架

五个包分别维护自己的package.xml、CMake和launch。/nav/cloud须按传感器时间转换到map；/nav/odom使用map与base_link约定。Gazebo入口已提供这一适配，不要仅修改frame_id代替实际变换。使用Gazebo时，地图、规划、控制及RViz都必须启用仿真时间。

接入其他环境时停止原来的同话题provider，确保地图、规划、控制及底盘指令没有重复发布者。ROS接口见[主引导](../README.md)；本作业不包含真机启动服务。
