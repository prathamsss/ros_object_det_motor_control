FROM ros:noetic-perception-focal
RUN rm /bin/sh && ln -s /bin/bash /bin/sh


# Update and install required things.
RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install wget
RUN apt-get update && apt-get install -y git

# Create a Catkin workspace and clone project repos
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && mkdir -p /catkin_ws/src \
 && cd /catkin_ws/src \
 && catkin_init_workspace \
 && git clone https://github.com/prathamsss/ros_object_det_motor_control.git \
 && pip3 install -r ros_object_det_motor_control/scripts/requirements.txt

# Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && cd catkin_ws \
 && catkin_make



# Set the working folder at startup
WORKDIR /catkin_ws
ENTRYPOINT [ "/entrypoint.sh" ]
