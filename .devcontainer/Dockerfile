ARG BASE_IMAGE=lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4

FROM ${BASE_IMAGE} as base

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get upgrade -y 
RUN rosdep update --rosdistro ${ROS_DISTRO}

FROM depbuilder as final

USER ros
