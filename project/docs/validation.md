# 验证记录：2026-10-27

本作业包的提供部分 **PASS**。学生算法故意保留 TODO；交付模板本身不声称已通过学生作业验收。

## 环境与构建

- Linux x86_64，Docker；Ubuntu 22.04 / ROS 2 Humble。
- 默认官方基础镜像 `ros:humble-ros-base`，本次解析为 `sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963`。
- 实际执行 `docker build -t trident-control-course:2026 .`，编译和镜像导出成功。见 [构建日志](evidence/docker-build.log)。默认 tag 将来可能更新；需要冻结环境时可通过 Dockerfile 的 `BASE_IMAGE` 参数使用上述 digest。
- Docker 构建内执行 CTest：`legacy_math` 通过，包含绕墙、封闭墙无解、负坐标拒绝、禁止对角穿角、重复求解、40 张固定种子的随机地图与独立 BFS 可达性核对、PID 前馈比例输出、积分限幅和重置。
- Python AST、package.xml、YAML 解析，Markdown 本地链接和 Git 空白检查通过。

## ROS / DDS 实测

命令均运行在独立容器内，分别使用 ROS domain 176 / 177 / 178；没有 Gazebo、真机、串口或雷达连接。

| 检查 | 实际结果 |
| --- | --- |
| `check_closed_loop.py --diagnostic` | 到达 20.80 s，位置误差 0.1791 m，最大命令速度 0.6500 m/s，无底盘碰撞，运动中断输入后持续零速度；PASS |
| 教师临时参考算法 + 五组公开地图单元测试 | 5 passed；PASS |
| 教师临时参考算法 + `check_closed_loop.py` | 点云经过投影和膨胀进入旧导航；到达 20.70 s，位置误差 0.1783 m，最大命令速度 0.6500 m/s，无碰撞，运动中断输入后停车；PASS |
| 原始学生模板 + 五组地图测试 | 5 failed，均为预期 `NotImplementedError`；没有偷填答案或跳过测试 |
| 原始模板默认 launch | 三节点启动，报告 TODO A；6 秒后发 SIGINT，三节点正常退出；timeout 返回 124 是测试主动结束，不是 launch 崩溃 |

日志：[提供部分闭环](evidence/diagnostic-closed-loop.log)、[教师参考闭环](evidence/reference-closed-loop.log)、[模板测试](evidence/template-tests.log)、[默认启动与退出](evidence/default-launch.log)。

教师参考实现由本地独立文件临时只读挂载到测试容器，**不在此仓库、不在交付镜像的任何层中**。完整点云链路通过只说明接口和验收环境可工作，不代表学生模板已完成。

## 改造与检查边界

源自旧工程的 D* Lite / LinkCutTree 和 PID 数学逻辑，适配文件见 [来源说明](provenance.md)。原地图生成逻辑被独立学生接口替代，没有运行 ROG-Map、衰减或射线遮挡模块。当前框架的输入输出沿用 `/nav/cloud`、`/nav/map`、`/nav/odom`、`/nav/goal`、`/nav/planning/path`、`/nav/cmd_vel`。

首轮手写导航测试图在墙体下侧留出过窄通道，导致跟踪进入膨胀栅格后按设计停车。已把教学场景中墙体延伸到下边界，并同时更新点云障碍和手写二维测试图，使验收明确测试墙体上方绕行。上述通过结果来自修正后的最终场景。退出流程对 ROS context 已关闭的正常信号路径进行了处理，最终日志无关闭异常。

未进行 Gazebo 动力学、真实 LiDAR、真车、坡道、移动障碍、其他 CPU 架构或真实比赛场地验证。此前验证使用完整世界地图快照和二维运动学环境；不能作为动态局部地图验收。新版本改为实时扫描和滚动窗口，新的验证见后续记录。

## 动态局部地图版本（2026-10-27）

此前完整世界点云验证仅为旧接口回归。新分支 `coursework/live-local-map` 的学生与教师 Docker 均已构建。DDS 动态输入验收通过障碍出现/移动/消失、无残留、膨胀、滚动原点、原始时间戳及旧扫描拒绝。范围裁剪点云的平面闭环21.32秒到达，误差0.1725米，无碰撞，限速与断流停车通过。对应日志见 evidence/live-map-acceptance.log 和 evidence/local-closed-loop.log。

另外执行真实 Gazebo 雷达 case-0001：地图动态更新与记录完整，但导航在100秒仿真后超时，机器人进入当前膨胀占据格后保持零速，未观察到物理接触。此项没有通过到点验收；不能将平面夹具或旧完整地图成功结果替代它。

## 半数柱子全局先验版本

新增提供的静态全局先验（60/120根柱子，实际世界仍为120根）及当前局部障碍融合。5项先验融合测试通过，学生/教师Docker构建成功。新的真实Gazebo case-0001成功：56.016秒、终点误差0.170029米、路程31.471559米，接触观测可用且未观察到物理碰撞，记录完整。167帧融合图均保留已知先验，雷达额外补充最多1219个占据格，局部窗口154个不同原点。此结果与上一节纯局部地图失败是不同运行，不改写旧结果。

## 分模块自由测试入口

新增 environment.launch.py、mapping.launch.py、navigation.launch.py。环境默认完整120柱森林、无自动goal；支持 scene:=wall，墙体场景需关闭森林先验。README第一节提供同一容器多终端启动与手动goal命令，自动脚本只放在最终验收节。森林是无界面的平面运动学夹具，5米范围裁剪合成扫描，不能称为真实Gazebo雷达。

本次只读挂载新launch、环境脚本和资源到仅本地教师参考镜像验证，没有构建学生镜像：启动后5秒没有goal也没有运动，融合地图持续发布；随后通过README原样ROS CLI命令手动发goal，60.45秒内到达、无运动学碰撞。原始日志见 evidence/manual-launch-acceptance.log。新增入口需重新构建学生Docker后才会安装；学生两个TODO仍未完成。

## 框架分包版本（当前）

先前章节是对应历史提交的单包验证。当前版本已移除 trident_course，独立构建 nav_obstacle_map、nav_planning、nav_legacy_control、nav_simulation、nav_bringup；路径见 README 和 architecture.md。规划与控制不再位于同一进程。

学生验证镜像 package-layout-check 与本地教师镜像 instructor-packages 均实际构建成功；colcon发现恰好五个包，规划、控制各自数学测试通过。学生镜像中调用投影仍抛出NotImplementedError，未混入教师答案。

教师参考实现验证：地图/先验10项测试通过，动态DDS地图测试通过；墙体闭环21.48秒到达、误差0.1731米、无运动学碰撞、断流停车通过。四个模块launch独立启动后没有自动goal、没有运动；手动ROS CLI森林目标50.17秒到达、无运动学碰撞。再次给返程目标，单独关闭规划包后，地图与里程计继续更新，控制仍因路径过期停车。日志见 evidence/packages-*.log。

本次未重新运行分包后的真实Gazebo物理benchmark。上一版56.016秒真实Gazebo结果保留为历史回归，不冒充本次分包验证。用户现有:2026旧镜像没有被覆盖，必须重建后才包含新包。
