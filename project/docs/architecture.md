# 框架分包与运行边界

教学仓库沿用母框架的 `src/nav/<component>` 逻辑目录；使用五个独立 ROS 包：地图 nav_obstacle_map、规划 nav_planning、控制 nav_legacy_control、环境 nav_simulation、启动编排 nav_bringup。包清单、依赖、CMake、launch、配置分别维护，不再包含 trident_course 聚合包。

学生文件位于 `src/nav/obstacle-map/nav_obstacle_map/python/nav_obstacle_map/student_map.py`。全局先验及其融合也属于地图包；完整120柱环境数据只由 simulation 包加载。规划包只链接旧 D* Lite，控制包只包含旧 PID；二者通过 `/nav/planning/path` 的几何 Path 通信。控制还独立检查地图、里程计与路径新鲜度，规划停止后不会无限跟踪旧路径。

`nav_bringup` 只包含依赖与launch编排，不实现算法。七组件映射在 config/components.yaml；课程中未用的真机组件由明确的仿真/手动输入替代，没有新增假实现或空 ROS 包。教学版为单 Git 仓库交付、多 ROS 包构建；不修改母工作区的生产独立组件仓库。

迁移前 → 迁移后：

- trident_course/python/.../student_map.py、mapper_node.py、prior_node.py → nav/obstacle-map/nav_obstacle_map
- trident_course/src/legacy_dstar.cpp → nav/planning/nav_planning
- trident_course/include/.../legacy_pid.hpp → nav/control/nav_legacy_control
- trident_course/python/.../demo_node.py 与完整世界资源 → nav/simulation/nav_simulation
- 总 launch → nav_bringup；模块 launch 属于各自业务包

原先合并的 navigation_node 被拆为 planner_node 和 controller_node；不是仅移动目录或增加 launch 别名。旧报告保留原提交对应的单包路径，不作为当前构建命令。当前有效操作以 README 为准。
