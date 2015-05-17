#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

class video_resizer:
    def __init__(self):
        self.sub_vid = rospy.Subscriber('/IR_data', Image, self.callback_ir)
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_color', Image, self.callback_rgb)
        self.pub_vid = rospy.Publisher('/IR_data_big',  Image, queue_size=100)
        self.bridge = CvBridge()
        self.ir_counter = 0
        self.rgb_counter = 0
        self.current_frame = None

    def callback_ir(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e
            return
       # print "heh"
        cv_image = cv2.resize(cv_image, (320, 240))
        #print cv_image.shape
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        self.pub_vid.publish(msg)
        #cv2.imwrite("/home/radek/catkin_ws/src/kinect_io/data/vid_sync/ir" + str(self.ir_counter) + ".jpg", cv_image)
       # cv2.imwrite("/home/radek/catkin_ws/src/kinect_io/data/vid_sync/rgb" + str(self.rgb_counter) + ".jpg", self.current_frame)
        self.ir_counter = self.ir_counter+1
        self.rgb_counter = self.rgb_counter+1

        #print "stuff"

    def callback_rgb(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e
            return
       # print "heh"
        cv_image = cv2.resize(cv_image, (640, 480))
        self.current_frame = cv_image
        #print cv_image.shape
        #msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")



        #print "stuff"



def listener():
    vr = video_resizer()
    rospy.init_node('video_resizer', anonymous=True)
    print "gkh"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
