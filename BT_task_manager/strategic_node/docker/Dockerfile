FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
RUN chmod 1777 /tmp

RUN apt-get update && \
    apt-get -y install --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends git \
                       python3-pip && \
    rm -rf /var/lib/apt/lists/*

ARG USER=docker_user
ARG UID=1000
ARG GID=1000
# default password
ARG PW=docker_user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} \
    && echo "${USER}:${PW}" | chpasswd \
    && usermod -s /bin/bash ${USER} \
    && usermod -a -G sudo ${USER}

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-actionlib-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends nano && \
    rm -rf /var/lib/apt/lists/*

RUN pip install git+https://github.com/splintered-reality/py_trees/@release/0.7.x \
                git+https://github.com/splintered-reality/py_trees_ros/@release/0.6.x \     
                termcolor \
                git+https://github.com/splintered-reality/rqt_py_trees/@release/0.4.x

WORKDIR /home/${USER}
