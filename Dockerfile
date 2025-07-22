FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install -r requirements.txt

WORKDIR /PMS_server
COPY ./src ./src

# Autoware 메시지 의존성 clone
COPY autoware_msgs_dependencies.repos .
RUN vcs import src < autoware_msgs_dependencies.repos

# rosdep 설치
RUN apt update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# FastAPI + ROS 실행 스크립트 복사
COPY start.sh /start.sh
RUN chmod +x /start.sh

# ROS2 패키지 빌드 (너의 패키지 포함)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
    --packages-select \
        autoware_common_msgs \
        tier4_system_msgs \
        ros2api_bridge

# 개발자 편의용 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /PMS_server/install/setup.bash" >> ~/.bashrc
