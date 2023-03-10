#!/usr/bin/env python3

"""
This node subscribes to all available image topics and applies background subtraction
to detect any bad objects in the camera's field of view.
"""


import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber:
    """
    Constructor for the ImageSubscriber class.

    :param topic_name: The name of the image topic to subscribe to.
    """
    def __init__(self, topic_name):

        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.window_name = self.topic_name
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(self.window_name, 640, 480)
        self.rate = rospy.Rate(10)
        self.image_sub = rospy.Subscriber(self.topic_name, 
                                          Image, self.callback,
                                          queue_size=100)
        self.motor_pub = rospy.Publisher('/motor_status', Bool, queue_size=1)
        # initialise object detection params.
        self.baground_sub = cv2.createBackgroundSubtractorMOG2()

    def callback(self, data):
        """
        Callback function that is called whenever a new image is received on the subscribed topic.

        :param data: The image data received on the subscribed topic.
        """

        # convert image from  ros to cv2 
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # get decision from below function for switching motor if object is present
        motor_decision = self.detect_bad_object(cv_image)
        self.motor_pub.publish(motor_decision)

        #  Set motor_status as True if object is detected.
        # print(motor_decision)
        # if motor_decision:
        #     rospy.set_param("motor_status",True)
        # else:
        #     rospy.set_param("motor_status",False)
       
    def detect_bad_object(self,image):
        """
        Applies background subtraction to the given image 
        and detects any bad objects. 
        For test example case: I have considered turning Motor.
        ON when person is in the frame.  

        :param image: The image for detecting object.
        :return: True if a bad object is detected, False otherwise.
        """
        # Convert image to gray scale.
        image  = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        # Get foreground seperate from background.
        fgmask =self.baground_sub.apply(image)

        # Make it blur
        fgmask = cv2.GaussianBlur(fgmask,(7,7),0)
        
        # It it shaped
        contours, _ = cv2.findContours(fgmask,cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        # Showing the output Image
        # cv2.imshow(self.window_name, fgmask)
        # cv2.waitKey(1)

        # Here if there are one or more objects return True. 
        if (len(contours)) >= 1:
            return True
        else:
            return False
    

def check_image_topics(timer_event):
    """ 
    This Method checks the update for new camera stream every 5 sec. 
    """
    all_image_topics = rospy.get_published_topics()
    print(all_image_topics)
    for topic, msg_type in all_image_topics:
        if topic.startswith('/camera/') and msg_type == "sensor_msgs/Image":
            print(f"Subscribing to {topic}")
            ImageSubscriber(topic)    

if __name__ == '__main__':
    try:
        rospy.init_node('object_detection_node', anonymous=True)
        
        rospy.loginfo("Starting object Detection Node...")

        rospy.Timer(rospy.Duration(5), check_image_topics)

        rospy.spin()

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass