FROM ubuntu:22.04

RUN apt update

WORKDIR /root

RUN apt install -y python3 \
    wget

# install pip
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3 get-pip.py
RUN rm get-pip.py

# install dependencies
RUN pip install asyncio
RUN pip install mavsdk
RUN pip install msgpack-rpc-python
RUN pip install numpy
RUN pip install airsim

# install ros2
RUN apt update
RUN apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y ros-humble-desktop ros-humble-ros-base

# copy the project
COPY src src

COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "bash", "-c", "./entrypoint.sh" ]
