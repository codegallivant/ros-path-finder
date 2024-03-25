# Creating a docker image of ros with c++ cmake catkin rosbash map-server and rviz preinstalled

FROM ros:noetic-ros-core

RUN apt-get -y update
# RUN apt-get install -y cmake
# RUN apt-get install -y g++
# RUN apt-get install -y python3-catkin-pkg
# RUN apt-get install -y catkin
# RUN apt-get install -y rosbash
RUN apt-get install -y make cmake g++
RUN apt-get -y update
RUN apt-get install -y ros-noetic-catkin
RUN apt-get install -y ros-noetic-rosbash
RUN apt-get -y update
RUN sudo apt-get install -y ros-noetic-map-server
RUN apt-get -y update
RUN sudo apt-get install -y ros-noetic-rviz