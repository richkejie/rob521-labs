FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

USER root

RUN apt-get update

RUN apt-get install -y build-essential sudo terminator iproute2 gedit lsb-release lsb-core wget nano vim
RUN apt-get install ros-noetic-rviz

RUN adduser user
RUN adduser user sudo

# remove password
RUN passwd -d user

USER user