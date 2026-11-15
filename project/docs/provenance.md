# 旧导航来源与适配边界

来源：内部仓库 https://github.com/trident-rm/2027-navi-demo ，commit `a0ffc16786ca5225022219fbb1f53fb952a5acdb`。这是一份当前保留旧 ROS 1 工程的来源快照，不声称它本身是去年比赛冻结的最终提交。原始文件 SHA256 见 `provenance.json`。

复用：

- `projects/hku-astar-ros1/sim_nav/src/bot_sim/src/dstarlite.cpp` 的 Node、LinkCutTree、rhs 更新、搜索队列、sigmoid 栅格成本；抽离为 `legacy_dstar.hpp/.cpp`。
- `.../include/bot_sim/velocity_pid_controller.hpp` 的 `updateAxis` 数值实现原样保留；包括前馈、P/I/D、积分限幅、死区与原始误差导数。提取为 `legacy_pid.hpp`。
- 保留六格前视与终点减速思路，ROS 2 节点重新封装消息、定时器和状态。

有意调整：

1. ROS 1 / TF 取位姿边界改成 ROS 2 `/nav/odom`，直接使用机体系速度反馈；不再差分 TF 估速。
2. 接入当前框架 `/nav/*` 话题。Path 是几何路径，不能不经适配就交给当前按时间戳跟踪的 NMPC。
3. 学生地图是当前雷达帧重建的滚动局部二维快照。每次规划从当前快照初始化并重新搜索，不保留原来跨回调的动态概率 / 衰减状态，当前适配不宣称保留完整增量重规划性能。
4. 占据和未知格作为硬障碍；禁用对角穿角，增加前视线段碰撞检查；LCT 根查找补充懒反转下传，初始化 `reversed`，路径追踪加循环上限。
5. 删除本作业不需要的原地图投影、代价扩散、概率更新、历史衰减、坡道增速与遮挡相关输入链路；题目代码内没有现成投影/膨胀答案。
6. 20 Hz 控制，5 Hz 独立规划；默认限速 0.65 m/s、加速度 1 m/s²，航向保持不变。限速与限加速度应用在 PID 输出后。停车立即归零，停车不受舒适性加速度限制。
7. 地图/里程计过期、时钟异常、frame 不符、无路径时发布零速度；目标完成半径 0.18 m。

来源仓库未提供覆盖上述 bot_sim 文件的明确开源授权，本教学派生包按内部资料处理；不额外授予上游代码开源许可，保留已有作者注释。远程仓库保持 private。公开发布前由维护者核对来源权限。

框架分包后，数学核心分别位于 nav/planning/nav_planning 与 nav/control/nav_legacy_control。只修改包含路径和ROS封装边界；旧 PID 数值头文件保持不变。规划与控制节点分别构建，通过几何Path话题通信。原始来源哈希仍在 provenance.json，不将新ROS包装误称为旧源码原样复刻。
