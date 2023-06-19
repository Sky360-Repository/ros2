FROM sky360/ros2-humble-dev:latest

ENV DEBIAN_FRONTEND=noninteractive

# Removing this ros dependencies as they use old OpenCV
RUN apt-get update && apt-get -y install ros-humble-rqt* ros-humble-rqt-image-view
RUN apt-get -y remove ros-humble-vision-opencv ros-humble-cv-bridge

# Building new ros-humble-vision-opencv
RUN mkdir -p /opt/ros2_ws/src
WORKDIR /opt/ros2_ws
RUN git clone https://github.com/ros-perception/vision_opencv.git
RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro humble -y
RUN colcon build
# RUN source install/setup.bash

RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-numpy libboost-python-dev \
   # Install colcon build tools
   && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
   # ROS2 build related stuff
   && pip3 install setuptools==58.2.0 \
   && export PYTHONWARNINGS=ignore:::setuptools.command.install \
   && apt-get autoremove -y \
   && apt-get clean -y
   # && rm -rf /var/lib/apt/lists/*

# RUN pip3 install tensorflow

# RUN apt-get update && apt-get -y install vulcanexus-humble-base vulcanexus-humble-base

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f /opt/vulcanexus/humble/setup.bash ]; then source /opt/vulcanexus/humble/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# ARG USERNAME=ros
# RUN chown -R $USERNAME:$USERNAME /opt/Fast-DDS-statistics-backend