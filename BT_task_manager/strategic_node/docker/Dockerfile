FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
RUN chmod 1777 /tmp

RUN apt-get update && \
    apt-get -y install --no-install-recommends \
        tzdata \
        git \
        python3-pip \
        ros-noetic-actionlib-tools && \
    rm -rf /var/lib/apt/lists/*

ARG USER=docker_user
ARG UID=1000
ARG GID=1000

RUN groupadd ${USER} --gid=${GID} \
    && useradd -m ${USER} --uid=${UID} --gid=${GID} \
    && usermod -s /bin/bash ${USER} \
    && usermod -a -G sudo ${USER} \
    && echo '%sudo   ALL=(ALL:ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /home/${USER}
