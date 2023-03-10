#!/usr/bin/env python3
"""
Utility to make multiple publisher for multiple cameras.
It uses rtsp link. 
We use MongoDB database, where RTSP links are stored.
(For Test purpose- we use Video File as camera stream.) 
"""

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospy
from database.db_utils import MyMongoDB


class CameraPublisher:
    
    def __init__(self):

        # Initialize the ROS Node
        rospy.init_node('video_pub_py', anonymous=True)
        
        self.database = MyMongoDB("camera_devices", "factory_1")
      
        self.rate = rospy.Rate(100)  # 100hz

        self.br = CvBridge()

        self.video_cap = {}

    def register_cam(self):
        """
        Method to register cameras using the database and 
        create publishers
        """
        data = self.database.retrieve_data()

        for each_camera in data:
            topic_name  = "/camera/" + str(each_camera["cam_id"])
            rtsp_link = str(each_camera["rtsp_link"])
            pub = rospy.Publisher(topic_name, Image, queue_size=10)
            self.video_cap[cv2.VideoCapture(rtsp_link)] = (pub)

        rospy.loginfo("Total of camera streams to Publish : %s",
                      len(self.video_cap))
            

    def publish_message(self):
        """
        This method reads Video file (Assume that this would be RTSP camera link).
        It reads frames using Opencv, converts into ROS formate using CvBridge,
        and publishes frames to approprate topic.

        """
        
        # While ROS is still running.
        while not rospy.is_shutdown():

            for each_cap,each_pub in self.video_cap.items():
                ret, frame = each_cap.read()

                if ret == True:

                    frame = cv2.resize(frame, (640, 480))

                    each_pub.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))
    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                else:
                    # Set the video capture object position to 0
                    # to ensure the video does not end
                    each_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

            # Sleep just enough to maintain the desired rate
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting Camera Publisher ")
        cam = CameraPublisher()
        cam.register_cam()
        cam.publish_message()
    except rospy.ROSInterruptException:
        pass
