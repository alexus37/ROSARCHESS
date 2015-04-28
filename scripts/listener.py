#!/usr/bin/env python
from OpenGL.raw.GLUT import glutPostRedisplay
import rospy
import cv2
import numpy as np
from samba.dcerpc.dns import res_rec
import tf
import sys
import os
import threading

sys.path.insert(0, "/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
sys.path.insert(0, "/home/radek/catkin_ws/src/kinect_io/aruco_python_wrapper/build")
import GameNoLogic
import libGetGlCamera as cam

from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError


# noinspection PyPep8Naming
class kinect_listener:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)

        # intialize python aruco wrapper
        cam.init()

        self.display_stuff = 0
        if self.display_stuff:
            cv2.namedWindow("RGB window", 1)
            cv2.namedWindow("DEPTH window", 1)

        self.game = GameNoLogic.Game()

        self.bridge = CvBridge()
        # self.currentFrame = 0
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback_rgb)
        self.sub_depth = rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)
        #self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)
        self.sub_modelview = rospy.Subscriber('ar_single_board/modelview', Float32MultiArray, self.callback_modelview)
        self.sub_cam_info = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.callback_cam)
        self.sub_ir = rospy.Subscriber('/IR_data', Image, self.callback_ir)


        self.P = None
        self.imX = 640
        self.imY = 480

        # DO EVERYTHING BEFORE THIS POINT!!!
        os.chdir("/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
        # thread.start_new_thread(self.start_game,())
        # self.game.start()
        thread = threading.Thread(target=self.start_game, args=())
        thread.daemon = False
        thread.start()



    def callback_cam(self, data):
        if self.P is None:
            # print data.P
            self.P = np.array(data.P)
            self.P = np.reshape(self.P, [3, 4])


    def loadCamera(self):
        if self.P is not None:
            # print self.P
            fx = self.P[0][0]
            cx = self.P[0][2]
            fy = self.P[1][1]
            cy = self.P[1][2]
            # print fy


            glP = cam.getGlCamera(fx,cx,fy,cy,self.imX, self.imY, self.game.width, self.game.height)
            glP = glP.flatten()
            self.game.projection = glP.view()

    def callback_ir(self,data):
        # print "IR image arrived"
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
        print keypoints
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        #resized = cv2.resize(im_with_keypoints, (0,0), fx=10, fy=10)
        cv2.resizeWindow("Keypoints",400,400)
        cv2.imshow("Keypoints", im_with_keypoints)
        self.video.write(im_with_keypoints)
        #cv2.waitKey(0)

    def callback_pose(self, data):
        if not self.game.ready:
            return

        self.loadCamera()
        glutPostRedisplay()

    def callback_modelview(self, data):
        self.loadCamera()
        self.game.modelview = np.array(data.data).flatten() # array(data.data, dtype=float)
        glutPostRedisplay()

    def callback_position(self, data):
        print "incoming position!"

    def callback_transform(self, data):
        print "incoming transform!"


    def callback_rgb(self, data):
        if self.game.ready:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
                # self.currentFrame = cv_image
                #resizedFrame = cv_image
                #print "-----------"
                #print cv_image.shape

                cvHeight, cvWidth, _ = cv_image.shape
                #print str(self.game.height) + ", " + str(self.game.width)
                #print str(cvHeight) + ", " + str(cvWidth)

                #newX = float(self.game.width) / float(cvWidth)
                #newY = float(self.game.height) / float(cvHeight)
                #print "scaleX = " + str(newX)
                #print "scaleY = " + str(newY)
                #resizedFrame = cv2.resize(cv_image, (0, 0), fx=newX, fy=newY)
                #self.imY, self.imX, _ = resizedFrame.shape
                # self.game.currentFrame = resizedFrame

                self.imY, self.imX, _ = cv_image.shape
                self.game.currentFrame = cv_image

                #print resizedFrame.shape

                self.game.newFrameArrived = True
                glutPostRedisplay()
            except CvBridgeError, e:
                print e
            if self.display_stuff:
                cv2.imshow("RGB window", cv_image)
                cv2.waitKey(3)

    def callback_depth(self, data):
        try:
            cv_image_depth = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e
        if self.display_stuff:
            cv2.imshow("DEPTH window", cv_image_depth)
            cv2.waitKey(3)

    def start_game(self):
        self.game.start()


def listener():
    kl = kinect_listener()
    rospy.init_node('kinect_listener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
