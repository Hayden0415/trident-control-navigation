# Third-party source and asset notices

Upstream source: https://github.com/trident-rm/Omni_sim
Pinned upstream commit: 2c64cee25776b361714ac8af5011acf3d39e2b56.

The benchmark reuses these upstream materials without replacing their provenance:

- `rmoss_gazebo/rmoss_gz_plugins`: OmniDrive and other Gazebo plugins, copyright
  RoboMaster-OSS as stated in their source headers. The upstream
  `rmoss_gazebo/LICENSE` is Apache License2.0 and is installed with the plugin package.
- `rmoss_gz_resources/resource/models/sentry`: robot description and CAD meshes.
  The upstream `rmoss_gz_resources/LICENSE` is Apache License2.0 and is installed
  with that resource package.
- `rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2025` and
  `resource/worlds/rmuc_2025_world.sdf`: exact upstream2025UC scene and STL mesh.
  The upstream `rmu_gazebo_simulator/LICENSE` is Apache License2.0. Its verbatim
  text is included as `UPSTREAM_SCENE_LICENSE` beside this notice.

New benchmark adapter code is declared Apache-2.0 in package.xml. The included
LICENSE is the upstream Apache-2.0 text, retained verbatim. This record reports
repository declarations and file origins; it does not assign missing asset-specific
copyright or infer additional rights for files without separate attribution.

All benchmark profile changes are generated at runtime and described in README.md.
Original plugin source copyright/license headers are retained.

The pillar-only scene is adapted from ZJU-FAST-Lab/ego-planner at
bfda51284c8c1b476043255a8145ef925a3778a5, specifically
src/uav_simulator/map_generator/src/random_forest_sensing.cpp.
The original source, GPL-3.0 license and provenance are retained in
third_party/ego_forest. nav_simulation/forest.py is a GPL-3.0-only derivative.
It uses seeded PCG64 sampling, fixed solid-cylinder heights and physical floor/
walls rather than claiming byte-identical upstream C++ random-device output.
The frozen JSON layout supplies simulator geometry only. Planners receive
simulated lidar through ROG-Map; the offline endpoint reference is not published.

`ego_pillar_forest_slim` is a separately named controlled derivative of that
frozen layout. All 120 ordered x/y centers, heights, layout seed and field/floor/
wall geometry remain unchanged; radii and diameters are uniformly multiplied by
0.85, without resampling. The original layout is retained. The companion
`resource/worlds/ego_pillar_forest_slim.SOURCE.json` records the parent simulation
commit and parent/new layout SHA-256 values. This derivative and its reference
do not replace old build/report geometry or supply an online obstacle oracle.
