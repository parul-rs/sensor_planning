FROM osrf/ros:jazzy-simulation-noble
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt update && apt install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    wget \
    && rm -rf /var/lib/apt/lists/*


# Core system + ROS packages
# osrf/ros:jazzy-simulation-noble already includes:
#   gz-harmonic, ros-jazzy-ros-gz-bridge, ros-jazzy-ros-gz-sim
#
# NOTE: python3-gz-transport13 / python3-gz-msgs10 are intentionally NOT
# installed here. They don't exist in the standard Noble/ROS apt repos for
# Jazzy — only on packages.osrfoundation.org, which would conflict with the
# vendored gz packages already in this image. The dynamics nodes shell out
# to the `gz service` CLI instead (ships with gz-harmonic, no extra package
# needed).
RUN apt update && apt upgrade -y \
    && apt install -y \
    apt-utils \
    nano \
    tmux \
    x11-apps \
    python-is-python3 \
    python3-pip \
    software-properties-common \
    ros-dev-tools \
    # Intel GPU / Mesa for Ogre2 (hyperon has Intel integrated graphics)
    # NOTE: libgl1-mesa-glx was removed in Noble; libglx-mesa0 replaces it.
    libglx-mesa0 \
    libgl1-mesa-dri \
    mesa-utils \
    libgles2-mesa-dev \
    libegl1-mesa-dev \
    # ROS control stack
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    # UR arm
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-msgs \
    # Camera image pipeline
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-vision-opencv \
    # RViz
    ros-${ROS_DISTRO}-rviz2 \
    # CycloneDDS — matches Jetson Orin default typically
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python packages used by dynamics nodes
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    transforms3d

# rosdep pre-install
# Copies src/ from sensor_planning (build context) so rosdep can
# resolve and install any package.xml-declared deps that aren't
# covered above. Src is deleted, live workspace is bind-mounted
WORKDIR /tmp_ws
COPY src ./src

RUN apt update \
    && apt dist-upgrade -y \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN rm -rf /tmp_ws

# Shell environment
WORKDIR /root

SHELL ["/bin/bash", "-l", "-c"]

RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' "/root/.bashrc" \
    && sed -i 's/01;32m/01;36m/g; s/01;34m/01;35m/g' "/root/.bashrc" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.profile \
    && echo 'if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Entrypoint
COPY gazebo_docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]