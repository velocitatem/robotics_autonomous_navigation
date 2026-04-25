FROM ros:noetic-robot-focal

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    git \
    python3-opencv \
    python3-pip \
    python3-pyqt5 \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-actionlib \
    ros-noetic-amcl \
    ros-noetic-controller-manager \
    ros-noetic-cv-bridge \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gmapping \
    ros-noetic-image-transport \
    ros-noetic-map-server \
    ros-noetic-message-filters \
    ros-noetic-move-base \
    ros-noetic-move-base-msgs \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-navfn \
    ros-noetic-navigation \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-sensor-msgs \
    ros-noetic-slam-toolbox \
    ros-noetic-smach \
    ros-noetic-smach-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-twist-mux \
    ros-noetic-velocity-controllers \
    ros-noetic-xacro \
    ros-noetic-image-geometry \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi || true
RUN rosdep update || true

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
