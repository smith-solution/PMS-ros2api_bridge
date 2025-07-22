FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install -r requirements.txt

WORKDIR /PMS_server

COPY ./src ./src

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /PMS_server/install/setup.bash" >> ~/.bashrc
# 1. docker compose 파일에서도 source 직접 하기 때문에 bashrc에 굳이 넣을필요없음.
# 2. 단 위 두줄이 의미있는 경우는, 개발자가 직접 docker exec -it <컨테이너ID> bash 로 접속했을때,
# -  bashrc에 두 줄 들어가고, docker exec에서 source ~/.bashrc 를 자동으로 해줌.

