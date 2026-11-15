ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | gpg --batch --yes --dearmor -o /usr/share/keyrings/course-gazebo.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/course-gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/course-gazebo.list
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-numpy python3-pytest \
    ros-humble-ament-cmake-python ros-humble-rclcpp ros-humble-rclpy \
    ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-std-msgs \
    ros-humble-sensor-msgs ros-humble-sensor-msgs-py ros-humble-launch-ros \
    ignition-fortress libignition-gazebo6-dev ros-humble-ros-gz-bridge \
    ros-humble-robot-state-publisher ros-humble-xacro ros-humble-tf2-ros-py \
    ros-humble-rviz2 libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/nav_ws
COPY src /opt/nav_ws/src
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && ctest --test-dir build/nav_planning --output-on-failure \
    && ctest --test-dir build/nav_legacy_control --output-on-failure
COPY scripts /opt/nav_ws/scripts
COPY config /opt/nav_ws/course-docs/config
COPY README.md /opt/nav_ws/course-docs/README.md
COPY docs /opt/nav_ws/course-docs/docs
COPY docker-entrypoint.sh /course-entrypoint.sh
ENTRYPOINT ["/course-entrypoint.sh"]
CMD ["ros2", "launch", "nav_bringup", "course.launch.py"]
