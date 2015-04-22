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
        self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)
        self.sub_modelview = rospy.Subscriber('ar_single_board/modelview', Float32MultiArray, self.callback_modelview)
        self.sub_cam_info = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.callback_cam)

        self.P = None
        # self.px = 0
        # self.py = 0

        # self.sub_transform = rospy.Subscriber('/ar_single_board/transform', TransformStamped , self.callback_transform)
        # self.sub_position = rospy.Subscriber('/ar_single_board/position', Vector3Stamped , self.callback_position)

        # DO EVERYTHING BEFORE THIS POINT!!!
        os.chdir("/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
        # thread.start_new_thread(self.start_game,())
        # self.game.start()
        thread = threading.Thread(target=self.start_game, args=())
        thread.daemon = False
        thread.start()



    def callback_cam(self, data):
        if self.P is None:
            #print data.P
            self.P = np.array(data.P)
            self.P = np.reshape(self.P, [3, 4])
            #print self.P
            # self.Pinv = np.matrix(np.linalg.pinv(P))
            #
            # K = np.array(data.K)
            # K = np.reshape(K, [3, 3])
            # print K
            # self.px = K[0][2]
            # self.py = K[1][2]


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



    # def glGetProjection(self, size):
    #     Ay = size[0]/self.game.height
    #     Ax = size[1]/self.game.width
    #     # TODO: find out why do we do this twice
    #     fx = P[0][0]*Ax
    #     cx = P[0][2]*Ax
    #     fy = P[1][0]*Ay
    #     cy = P[1][2]*Ay
    #     glP = [
    #             [fx, 0,  xc, 0],
    #             [0,  fy, cy, 0],
    #             [0,  0,  1,  0]
    #             ]


    def callback_pose(self, data):

        if not self.game.ready:
            return

        self.loadCamera()

        # q = data.pose.orientation
        # euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # matrix = euler_matrix(euler[0], euler[1], euler[2])
        # pos = data.pose.position
        # pos = [[-pos.x], [-pos.y], [-pos.z], [1]]
        # pos = np.matrix(pos)
        # # print "========="
        # # print np.transpose(pos)
        # # print np.matrix(euler)*180/np.pi
        #
        # #matrix[0][3] = -pos.x
        # #matrix[1][3] = -pos.y
        # #matrix[2][3] = -pos.z
        # inv = np.matrix(inverse_matrix(matrix))
        #
        #  worldPos = np.array( inv * pos)
        #

        # if self.Pinv is not None:
        #     #print "============"
        #     lookAt2D = np.matrix([[self.px], [self.py], [1]])
        #     #print self.Pinv
        #     #print lookAt2D.shape
        #     lookAtCamFrame = self.Pinv * lookAt2D
        #     # print lookAtCamFrame
        #     #lookAtCamFrame = lookAtCamFrame/lookAtCamFrame[-1]
        #     #print lookAtCamFrame
        #     lookAtWorld = inv * lookAtCamFrame
        #     self.game.lookAt = lookAtWorld[0:4]  #/lookAtWorld[4]
        #     #print self.game.lookAt
        #
        #
        # #cam_or = np.array([[0], [0], [0], [1]])
        # #cam_or = np.dot(matrix, cam_or)
        # #print matrix
        # #print cam_or
        # #cam_or[0:3] = cam_or[0:3]/cam_or[3]
        #
        # #print np.transpose(worldPos)
        #
        # up = np.matrix([[0.0], [1.0], [0.0], [0.0]])
        # trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [pos[0], pos[1], pos[2], 1]])
        # upTrans = inv * trans * up
        #
        # newCamPos = worldPos[0:3]
        # newCamUp = -upTrans[0:3]
        #
        # delta = abs((newCamPos - self.game.camPos).sum())
        #
        # if delta > 0.025:
        #     self.game.camPos = newCamPos
        #     self.game.up = newCamUp
        # #
        # #     F = -np.array([newCamPos.item(0), newCamPos.item(1), newCamPos.item(2) ] )
        # #     f = F / np.linalg.norm(F)
        # #
        # #     s = np.cross(f, np.array([0, 0, 1])) #np.array(up[0:3]))
        # #     s2 = np.array(s/np.linalg.norm(s))
        # #     u = np.cross(s2,  f)
        # #     u = u / np.linalg.norm(u)
        # #
        # #     M =  np.matrix(
        # #                     [ [ s2[0], s2[1], s2[2], 0 ],
        # #                       [ u[0], u[1], u[2], 0 ],
        # #                       [-f[0],-f[1],-f[2], 0],
        # #                       [0, 0, 0, 1] ])
        # #
        # #     T = np.matrix([ [1, 0, 0, -newCamPos[0]],
        # #                     [0, 1, 0, -newCamPos[1]],
        # #                     [0, 0, 1, -newCamPos[2]],
        # #                     [0, 0, 0, 1] ])
        # #     self.game.cameraMatrix = T*M
        # #     self.game.invCameraMatrix = numpy.linalg.inv(self.game.cameraMatrix)

        glutPostRedisplay()

    def callback_modelview(self, data):
        #self.game.modelview = data.data
        print type(data.data)
        self.game.modelview = np.array(data.data).view() # array(data.data, dtype=float)
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
                #print cv_image
                #cv2.cvtColor()
                #resizedFrame = cv_image
                #print "-----------"
                #print cv_image.shape

                cvHeight, cvWidth, _ = cv_image.shape
                #print str(self.game.height) + ", " + str(self.game.width)
                #print str(cvHeight) + ", " + str(cvWidth)

                newX = float(self.game.width) / float(cvWidth)
                newY = float(self.game.height) / float(cvHeight)
                #print "scaleX = " + str(newX)
                #print "scaleY = " + str(newY)
                resizedFrame = cv2.resize(cv_image, (0, 0), fx=newX, fy=newY)
                self.imY, self.imX, _ = resizedFrame.shape

                #print resizedFrame.shape
                self.game.currentFrame = resizedFrame
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
