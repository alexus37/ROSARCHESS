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
from arucoTest import *
import libGetGlCamera as cam

from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError


# noinspection PyPep8Naming
class kinect_listener:
    def __init__(self):
        cam.init()

        self.display_stuff = 0
        if self.display_stuff:
            cv2.namedWindow("RGB window", 1)
            cv2.namedWindow("DEPTH window", 1)

        self.game = Game()

        self.bridge = CvBridge()
        # self.currentFrame = 0
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback_rgb)
        #self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)
        self.sub_modelview = rospy.Subscriber('/ar_single_board/modelview', Float32MultiArray, self.callback_modelview)
        self.sub_cam_info = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.callback_cam)

        self.P = None


        # DO EVERYTHING BEFORE THIS POINT!!!
        os.chdir("/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")

        thread = threading.Thread(target=self.start_game, args=())
        thread.daemon = False
        thread.start()

    def callback_cam(self, data):
        if self.P is None:
            self.P = np.array(data.P)
            self.P = np.reshape(self.P, [3, 4])


    def loadCamera(self):
        if self.P is not None:
            # 548.1624862831703, 0, 299.7460070522235, 0, 534.7137541246177, 288.4127287793003, 0, 0, 1
            fx = self.P[0][0]
            cx = self.P[0][2]
            fy = self.P[1][1]
            cy = self.P[1][2]
            # fx = 548.1624862831703
            # cx = 299.7460070522235
            # fy = 534.7137541246177
            # cy = 288.4127287793003
            glP = cam.getGlCamera(fx, cx, fy, cy, self.imX, self.imY, self.game.windowSize[0], self.game.windowSize[1])
            # print "Projection matrix"
            # print glP
            glP = glP.flatten()
            # print "Flattened projection matrix"
            # print glP

            # print "======================"
            # print "PROJECTION MATRIX"
            # for i in glP:
            #     sys.stdout.write(str(i) + " ")
            # print "=========================="

            self.game.proj_matrix = glP.view()


    def callback_modelview(self, data):
        self.loadCamera()
        MV = np.array(data.data).reshape(4, 4)
        MV = MV.T

        self.game.modelview_matrix = MV.T.flatten() #data.data #Test.T.flatten()
        glutPostRedisplay()


    def callback_rgb(self, data):
        if self.game.ready:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
                cvHeight, cvWidth, _ = cv_image.shape

                newX = float(self.game.windowSize[0]) / float(cvWidth)
                newY = float(self.game.windowSize[1]) / float(cvHeight)

                resizedFrame = cv2.resize(cv_image, (0, 0), fx=newX, fy=newY)
                self.imY, self.imX, _ = resizedFrame.shape
                self.game.TheResizedImage = resizedFrame
                glutPostRedisplay()
            except CvBridgeError, e:
                print e
            if self.display_stuff:
                cv2.imshow("RGB window", cv_image)
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
