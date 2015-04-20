#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import tf 
import sys 
import os


from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

# noinspection PyPep8Naming
class ir_listener:

  def __init__(self):

    self.display_stuff = 1
    if self.display_stuff:
        cv2.namedWindow("IR window", 1)

    self.bridge = CvBridge()
    self.currentFrame = 0
    self.sub_ir = rospy.Subscriber('/IR_data', Image, self.callback_ir)


    self.video = None

    print "IR listener initialized"


  def callback_ir(self,data):
    #print "IR image arrived"
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data)
        self.currentFrame = cv_image
    except CvBridgeError, e:
        print e
        return
    if self.display_stuff:
        cv2.imshow("IR window", cv_image)
        cv2.waitKey(3)

    if self.video is None:
        self.video = cv2.VideoWriter('~/catkin_ws/blob_detection.avi',-1,1,cv_image.shape)

    # Set up the detector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    #params.minThreshold = 80
    #params.maxThreshold = 255

    params.filterByConvexity = True
    params.minConvexity = 0.80
    params.maxConvexity = 1.0

    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.maxCircularity = 1

    detector = cv2.SimpleBlobDetector(params)

    # Detect blobs.
    cv_image = cv2.resize(cv_image, (0,0), fx=10, fy=10)
    keypoints = detector.detect(cv_image)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    #resized = cv2.resize(im_with_keypoints, (0,0), fx=10, fy=10)
    cv2.resizeWindow("Keypoints",400,400)
    cv2.imshow("Keypoints", im_with_keypoints)
    self.video.write(im_with_keypoints)
    #cv2.waitKey(0)



def listener(): 
    irl = ir_listener()
    rospy.init_node('kinect_listener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
  	cv2.destroyAllWindows()
    irl.video.release()

    rospy.spin() 
    cv2.destroyAllWindows()

if __name__ == '__main__': 
    listener()
