FROM ros:noetic-perception-focal

WORKDIR /catkin_ws

COPY . /catkin_ws

RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install -r /catkin_ws/requirements.txt

ENV ROS_MASTER_URI=http://localhost:11311


CMD [ "rosrun" "motor_control" "camera_publisher.py" ]


docker run -it \
   --network="host" \
   --env="ROS_IP"=$ROS_IP \
   --env="ROS_MASTER_URI"=$ROS_MASTER_URI \
   ros:noetic-perception-focal

