FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|http://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list && \
    printf 'Acquire::Retries "8";\nAcquire::http::Timeout "20";\nAcquire::https::Timeout "20";\n' > /etc/apt/apt.conf.d/99-retry && \
    apt-get update && apt-get -y --fix-missing install \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    build-essential \
    cmake \
    git \
    colcon \
    python3-colcon-ros \
    python3-colcon-cmake \
    python3-colcon-package-selection \
    python3-colcon-parallel-executor \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy apt source (robust download with retries and mirror fallback)
RUN set -eux; \
    ROS_KEY_PATH=/usr/share/keyrings/ros-archive-keyring.gpg; \
    (curl -fL --retry 8 --retry-all-errors --retry-delay 2 --connect-timeout 10 \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ${ROS_KEY_PATH} || \
     curl -fL --retry 8 --retry-all-errors --retry-delay 2 --connect-timeout 10 \
      https://ghproxy.com/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ${ROS_KEY_PATH}); \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS_KEY_PATH}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get -o Dpkg::Options::="--force-overwrite" -y --fix-missing install \
    ros-jazzy-ros-base \
    ros-jazzy-ament-cmake \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rosidl-default-generators \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Build SDK-compatible fmt/spdlog into isolated prefix.
# - fmt v8 matches symbols like fmt::v8::...
# - spdlog built with external fmt to avoid system fmt ABI conflicts.
WORKDIR /opt/src
RUN curl -fL --retry 8 --retry-all-errors --retry-delay 2 \
      https://codeload.github.com/fmtlib/fmt/tar.gz/refs/tags/8.1.1 \
      -o fmt-8.1.1.tar.gz && \
    tar -xzf fmt-8.1.1.tar.gz && \
    mv fmt-8.1.1 fmt && \
    cmake -S fmt -B fmt/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/third_party \
      -DBUILD_SHARED_LIBS=ON && \
    cmake --build fmt/build -j"$(nproc)" && \
    cmake --install fmt/build

RUN curl -fL --retry 8 --retry-all-errors --retry-delay 2 \
      https://codeload.github.com/gabime/spdlog/tar.gz/refs/tags/v1.9.2 \
      -o spdlog-1.9.2.tar.gz && \
    tar -xzf spdlog-1.9.2.tar.gz && \
    mv spdlog-1.9.2 spdlog && \
    cmake -S spdlog -B spdlog/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/third_party \
      -DSPDLOG_FMT_EXTERNAL=ON \
      -DCMAKE_PREFIX_PATH=/opt/third_party && \
    cmake --build spdlog/build -j"$(nproc)" && \
    cmake --install spdlog/build

ENV CMAKE_PREFIX_PATH=/opt/third_party:/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/third_party/lib:/opt/ros/jazzy/lib

WORKDIR /work
