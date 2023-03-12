FROM ros:noetic-perception-focal
RUN rm /bin/sh && ln -s /bin/bash /bin/sh


# Update and install required things.
RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install wget
RUN apt-get update && apt-get install -y git

# Make a ROS Workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && mkdir -p /catkin_ws/src \
 && cd /catkin_ws/src 
# NOW GET OUR PROJECT HERE!

# Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && cd catkin_ws \
 && catkin_make

RUN echo "source /turtlebot3_ws/devel/setup.bash" >> ~/.bashrc

# Set the working folder at startup
WORKDIR /catkin_ws

