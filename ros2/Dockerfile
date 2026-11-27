ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /workspace

COPY ros2/ ros2/
RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential cmake python3-colcon-common-extensions python3-rosdep \
    && (test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths ros2 --ignore-src --rosdistro "${ROS_DISTRO}" -y \
    && rm -rf /var/lib/apt/lists/*

COPY config/ config/
COPY sdk/ sdk/
COPY models/300mm/ models/300mm/
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --base-paths ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN printf '%s\n' '#!/bin/bash' 'set -e' 'source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
    'source /workspace/install/setup.bash' 'exec "$@"' > /topoarm_entrypoint \
    && chmod +x /topoarm_entrypoint
ENTRYPOINT ["/topoarm_entrypoint"]
CMD ["bash"]
