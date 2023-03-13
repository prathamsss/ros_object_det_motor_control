# OBJECT DETECTION BASED MOTOR CONTROLLER.

This is a ROS node for controlling a motor based on object detection. It works as:
 - Reads each RTSP link from MongoDB.
 - Creates respective Image publisher for each RTSP link.
 - Object detection on each image topic identified.
 - Publish motor status based on object detection
 - Controlling motor using Twist configuration.
 
## Requirements
   - ROS Noetic 
   - Python 3.9
   - pip3

## Installation:
1. Clone the repository into your ROS workspace.


3. Install dependencies.
 

        pip3 install -r scripts/requirements.txt

4. Install using docker:
   1. build docker
   
          docker build -t my_dock .
   2. start the docker:

          docker run -it  --network="host" --env="ROS_IP"=$ROS_IP  --env="ROS_MASTER_URI"=$ROS_MASTER_URI my_dock roslaunch motor_control main.launch
   

### Usage:

- db_utils.py :  Use to manage Database. 
- camera_publisher.py : reads RTSP/video file path from Mongodb database and publishes streams.
- object_detection.py : Subscribes to above published streams, performes object detection and publihes it's result to /motor_status topic.
- motor_control.py : Gets the decision from object detection and sets motor's parameters accordingly.
 
   
