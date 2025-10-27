FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-noetic-nmea-msgs \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    terminator \
    git \
    ssh \  
    vim \
    python3-pip \
    minicom \  
    nano \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/* 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user and add it to the dialout group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && usermod -aG dialout ${USERNAME} \
    && usermod -aG sudo ${USERNAME} 

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists*

# Create SSH directory structure for the user
RUN mkdir -p /home/${USERNAME}/.ssh && \
    chmod 700 /home/${USERNAME}/.ssh && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh

# Pre-populate known hosts for GitLab and other common services
RUN ssh-keyscan gitlab.com github.com >> /home/${USERNAME}/.ssh/known_hosts 2>/dev/null && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/known_hosts && \
    chmod 644 /home/${USERNAME}/.ssh/known_hosts

# Cloning DJI OnboardSDK
RUN mkdir /home/${USERNAME}/libs
WORKDIR /home/${USERNAME}/libs
RUN git clone --branch 3.9 https://github.com/dji-sdk/Onboard-SDK.git /home/${USERNAME}/libs/Onboard-SDK

# Compiling and building DJI SDK -> IT IS NECESSARY TO USE make djiosdk-core !!!
WORKDIR /home/${USERNAME}/libs/Onboard-SDK
RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    make djiosdk-core && \       
    make install

# Cloning DJI OnboardSDK ROS
RUN mkdir -p /home/${USERNAME}/dji_ws/src
WORKDIR /home/${USERNAME}/dji_ws/src
RUN git clone --branch 3.8 https://github.com/dji-sdk/Onboard-SDK-ROS.git /home/${USERNAME}/dji_ws/src/Onboard-SDK-ROS

# Install dependencies if needed
WORKDIR /home/${USERNAME}/dji_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Compile
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/dji_ws /home/${USERNAME}/libs

USER ${USERNAME}

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build

ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"

RUN mkdir -p -m 0600 ~/.ssh/ && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Perception
WORKDIR /home/${USERNAME}
RUN mkdir -p /home/${USERNAME}/ARIEL/src
RUN --mount=type=ssh git clone --branch add-static-tests git@gitlab.com:gscar-coppe-ufrj/ariel/ros/main-ariel/accessories-nodes/ariel_launch.git
RUN --mount=type=ssh git clone --branch landpad-pose-extraction git@gitlab.com:gscar-coppe-ufrj/ariel/ros/main-ariel/accessories-nodes/aruco_collector.git
RUN --mount=type=ssh git clone --branch opencv4 git@gitlab.com:gscar-coppe-ufrj/ariel/ros/main-ariel/accessories-nodes/my_flavor_of_aruco.git
RUN --mount=type=ssh git clone --branch test_version git@gitlab.com:gscar-coppe-ufrj/ariel/ros/main-ariel/accessories-nodes/drone_tf.git

RUN apt update
RUN apt upgrade -y
RUN apt install -y build-essential
RUN apt install -y python3-opencv

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Create a helper script for SSH setup
COPY --chown=${USERNAME}:${USERNAME} ssh-check.sh /home/${USERNAME}/ssh-check.sh
RUN chmod +x /home/${USERNAME}/ssh-check.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
