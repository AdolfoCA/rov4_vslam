# ROS 2 Humble image for the BlueROV2 Heavy sensor drivers.
#
# Structured so that the expensive layers (apt, pip) are cached and only the workspace
# rebuilds when you touch the source. The source is also bind-mounted by
# docker-compose.yaml, so day-to-day you edit on the host and re-run `colcon build`
# inside the container rather than rebuilding the image.

FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DEBIAN_FRONTEND=noninteractive

# --- System dependencies -----------------------------------------------------------
RUN apt-get update -q \
    && apt-get upgrade -q -y \
    && apt-get install -y --no-install-recommends \
    # build tooling
    build-essential \
    cmake \
    git \
    sudo \
    python3-pip \
    python3-dev \
    python3-yaml \
    # networking / debugging tools you will want when the tether misbehaves
    iputils-ping \
    iproute2 \
    net-tools \
    netcat-openbsd \
    # SSH to the multi-camera computer: `multicam shutdown` powers it off with a
    # password login (sshpass), the password read from data/.multicam_ssh.
    openssh-client \
    sshpass \
    tcpdump \
    socat \
    tmux \
    vim \
    # ROS message packages used by the drivers
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-calibration \
    ros-humble-rmw-fastrtps-cpp \
    # CycloneDDS: only used by `multicam` to call the multi-camera system's services,
    # which run on CycloneDDS and do not answer calls from Fast DDS.
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-message-filters \
    # recording and visualisation
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-bridge \
    ros-humble-rqt-image-view \
    ros-humble-rviz2 \
    # GStreamer: the camera node decodes the ROV's H.264 UDP stream through it.
    # plugins-good has rtpjitterbuffer/rtph264depay, libav has avdec_h264.
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    python3-gi \
    python3-gi-cairo \
    python3-gst-1.0 \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    # X11 forwarding for rviz2 / rqt
    xauth \
    libeigen3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# --- AQUA-SLAM dependencies ----------------------------------------------------------
# Stereo + DVL + gyro SLAM (ros2_ws/src/aqua_slam, ported from the ROS 1 original).
# g2o and DBoW2 are vendored in the package; everything else comes from here.
#   OpenCV contrib   ximgproc, for the (optional) dense stereo mapper
#   PCL, octomap     map point clouds and the octree
#   Boost            log, serialization (Atlas save/load), filesystem, iostreams
#   fmt, OpenSSL     fmt for Sophus, MD5 for the vocabulary checksum in System
RUN apt-get update -q \
    && apt-get install -y --no-install-recommends \
    wget \
    ca-certificates \
    libopencv-dev \
    libopencv-contrib-dev \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-octomap \
    ros-humble-octomap-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-srvs \
    libboost-log-dev \
    libboost-serialization-dev \
    libboost-filesystem-dev \
    libboost-iostreams-dev \
    libboost-system-dev \
    libfmt-dev \
    libssl-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Sophus, header-only, the version AQUA-SLAM was developed against.
RUN git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cmake -S /tmp/Sophus -B /tmp/Sophus/build \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF \
    && cmake --install /tmp/Sophus/build \
    && rm -rf /tmp/Sophus

# ORB vocabulary (~145 MB unpacked). Kept in the image, not the repository; the
# launch file looks for it here.
RUN mkdir -p /opt/aqua_slam/Vocabulary \
    && wget -q -O /tmp/ORBvoc.txt.tar.gz \
       https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz \
    && tar -xzf /tmp/ORBvoc.txt.tar.gz -C /opt/aqua_slam/Vocabulary \
    && rm /tmp/ORBvoc.txt.tar.gz \
    && test -s /opt/aqua_slam/Vocabulary/ORBvoc.txt

# --- Python dependencies -----------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# --- Non-root user -----------------------------------------------------------------
# Matching the host UID/GID keeps files written into the bind-mounted workspace owned
# by you rather than by root.
RUN if getent group $GID >/dev/null; then \
        groupmod -n $USERNAME "$(getent group $GID | cut -d: -f1)"; \
    else \
        groupadd -g $GID $USERNAME; \
    fi \
    && useradd -m -u $UID -g $GID -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --- Middleware profile --------------------------------------------------------------
RUN mkdir -p /usr/local/share/middleware_profiles/
COPY rtps_udp_profile.xml /usr/local/share/middleware_profiles/

# --- Workspace -----------------------------------------------------------------------
COPY --chown=$UID:$GID ros2_ws/src /home/$USERNAME/ros2_ws/src

# `COPY --chown` sets ownership on ros2_ws/src only. The implicit parent directory
# /home/$USERNAME/ros2_ws is created by Docker as root:root, so the unprivileged user
# below cannot create build/, install/ or log/ in it and `colcon build` dies with
# `PermissionError: [Errno 13] Permission denied: 'log'`. Fix the parent while root.
RUN chown $UID:$GID /home/$USERNAME/ros2_ws

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# rosdep resolves the apt dependencies declared in each package.xml. Most are already
# installed above; this catches anything added to a package.xml later.
RUN sudo apt-get update -q \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/*

# AQUA-SLAM has translation units that need ~4 GB of RAM each to compile, so the
# build runs one compiler job at a time (~11 min for AQUA-SLAM). Raise it on a machine
# with more memory (budget 4 GB per job):
#   docker compose build --build-arg BUILD_JOBS=4
ARG BUILD_JOBS=1
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS=-j$BUILD_JOBS colcon build --symlink-install \
       --parallel-workers $BUILD_JOBS --cmake-args -DCMAKE_BUILD_TYPE=Release"

# --- Shell conveniences ----------------------------------------------------------------
RUN printf '%s\n' \
    'unbind C-b' \
    'set-option -g prefix C-a' \
    'bind-key C-a send-prefix' \
    'set -g mouse on' \
    'set -g default-terminal "screen-256color"' \
    'set -g history-limit 10000' \
    > /home/$USERNAME/.tmux.conf

RUN printf '%s\n' \
    "source /opt/ros/$ROS_DISTRO/setup.bash" \
    "[ -f /home/$USERNAME/ros2_ws/install/setup.bash ] && source /home/$USERNAME/ros2_ws/install/setup.bash" \
    "alias build_ws='cd /home/$USERNAME/ros2_ws && MAKEFLAGS=-j$BUILD_JOBS colcon build --symlink-install --parallel-workers $BUILD_JOBS --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash'" \
    "alias start_mission='bash /home/$USERNAME/ros2_ws/src/start_mission.sh'" \
    "alias stop_mission='tmux kill-session -t mission 2>/dev/null && echo Mission stopped.'" \
    "alias multicam='ros2 run bluerov2_bringup multicam.py'" \
    >> /home/$USERNAME/.bashrc

CMD ["bash"]
