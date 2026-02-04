FROM osrf/ros:noetic-desktop-full

# Arguments to pass during build (defaults to standard 1000:1000)
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=rob521

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle_pi

# 1. Install Dependencies
# Includes Lab 1 requirements + X11 utils for debugging
RUN apt-get update && apt-get install -y \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-laser-proc \
    ros-noetic-rgbd-launch \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python \
    ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-compressed-image-transport \
    ros-noetic-rqt* \
    ros-noetic-rviz \
    ros-noetic-gmapping \
    ros-noetic-navigation \
    ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \
    git \
    nano \
    sudo \
    mesa-utils \
    x11-utils \
    x11-apps \
    terminator \
    vim \
    && rm -rf /var/lib/apt/lists/*


# 2. Create User Matching Host
# CRITICAL: Add to 'video' (GPU) and 'dialout' (Serial) groups
RUN groupadd -g ${GROUP_ID} ${USER_NAME} && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USER_NAME} && \
    usermod -aG sudo,video,dialout ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 3. Setup Workspace
WORKDIR /workspace
RUN mkdir -p /workspace/src && chown -R ${USER_NAME}:${USER_NAME} /workspace

# Switch to non-root user
USER ${USER_NAME}

# 4. Install Simulation Packages (Source build for Lab 1)
WORKDIR /workspace/src
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# 5. Build Workspace
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 6. Configure Environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc