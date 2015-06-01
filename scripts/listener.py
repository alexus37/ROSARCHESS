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

# REPLACE THIS PATH WITH PATH TO THE GAME FOLDER
sys.path.insert(0, "/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
# REPLACE THIS PATH WITH PATH TO THE PYTHON ARUCO WRAPPER
sys.path.insert(0, "/home/radek/catkin_ws/src/kinect_io/aruco_python_wrapper/build")

import GameNoLogic
import libGetGlCamera as cam
import time


from tempfile import TemporaryFile
from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

# noinspection PyPep8Naming
class kinect_listener:
    def __init__(self):
        # intialize python aruco wrapper
        cam.init()

        self.display_stuff = 0
        if self.display_stuff:
            cv2.namedWindow("RGB window", 1)
            cv2.namedWindow("DEPTH window", 1)

        # initialize game object
        self.game = GameNoLogic.Game()

        # initialize CV bridge
        self.bridge = CvBridge()

        # initializie listeners
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback_rgb)
        self.sub_occlusion = rospy.Subscriber('/ar_single_board/occlusion_mask', Image, self.callback_occlusion)
        self.sub_depth = rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)
        #self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)
        self.sub_modelview = rospy.Subscriber('/ar_single_board/modelview', Float32MultiArray, self.callback_modelview)
        self.sub_cam_info = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.callback_cam)
        self.sub_ir = rospy.Subscriber('/IR_data', Image, self.callback_ir)
        self.sub_Rt = rospy.Subscriber('/ar_single_board/Rt', Image, self.callback_Rt)
        self.sub_K = rospy.Subscriber('/ar_single_board/K', Image, self.callback_K)

        # initialize rest of the stuff
        self.timestamp = 0
        self.currentFrame = None
        self.Rt = None
        self.P = None
        self.K = None
        self.Pcv = None
        self.imX = 640
        self.imY = 480

        # file for debug logs
        self.f = open('test.txt', 'w')

        # DO EVERYTHING BEFORE THIS POINT!!!
        os.chdir("/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
        thread = threading.Thread(target=self.start_game, args=())
        thread.daemon = False
        thread.start()



    def callback_cam(self, data):
        if self.P is None:
            # print data.P
            self.P = np.array(data.P)
            self.P = np.reshape(self.P, [3, 4])
            #print "K"
            #K = np.reshape(np.array(data.K), [3, 3])
            #np.save("/home/radek/catkin_ws/K.npy", K)


    def loadCamera(self):
        if self.P is not None:
            # print self.P
            fx = self.P[0][0]
            cx = self.P[0][2]
            fy = self.P[1][1]
            cy = self.P[1][2]
            # print fy

            glP = cam.getGlCamera(fx, cx, fy, cy, self.imX, self.imY, self.game.width, self.game.height)
            glP = glP.flatten()
            self.game.projection = glP.view()

    def callback_occlusion(self,data):
        self.game.flip = True
        #print "mask"
        if not self.game.ready:
            return
        try:
            occlusion_im = self.bridge.imgmsg_to_cv2(data)
            if occlusion_im.shape[0] == 0:
                return
            hand = np.zeros((self.imY, self.imX, 4))
            hand[:,:,0:3] = self.game.currentFrame
            hand[:,:,3] =  occlusion_im * 255
            self.game.currentHand = hand
            self.game.flip = 1
           # print hand.shape
            #cv2.imshow("mask window", hand)
            #print "mask"
        except CvBridgeError, e:
            print e
            return



    def callback_ir(self,data):
        # print "IR image arrived"
        if not self.game.ready:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            #self.currentFrame = cv_image
            cv2.imwrite("/home/radek/catkin_ws/ir_image.png", cv_image)
        except CvBridgeError, e:
            print e
            return
        if self.display_stuff:
            cv2.imshow("IR window", cv_image)
            cv2.waitKey(3)


        # Set up the detector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.blobColor = 255
        params.minThreshold = 80
        params.maxThreshold = 140

        #params.filterByConvexity = True
        #params.minConvexity = 0.80
        #params.maxConvexity = 1.0

        params.filterByCircularity = True
        params.minCircularity = 0.75
        params.maxCircularity = 1

        detector = cv2.SimpleBlobDetector(params)

        # Detect blobs.
        # (cv_image, (640, 480))
        # cv_image = cv2.resize(cv_image,(0,0), fx=10, fy=10) #(cv_image, (640,640)) #, interpolation=cv2.INTER_NEAREST)
        cv_image = cv2.resize(cv_image, (640,480))

        # IR CAMERA CALIBRATION MATRIX
        # K = np.matrix([[757.164773, 0, 335.313573], [0, 823.419897, 138.458047], [0, 0, 1]])
        #
        # # IR CAMERA DISTORTION COEFFICIENTS
        # distortion = np.matrix([-0.321378, -0.018411, -0.012105, -0.003352, 0])
        #
        # # UNDISTORT THE IR IMAGE
        # cv_image = cv2.undistort(cv_image, K, distortion)

        # BLOB DETECTION
        keypoints = detector.detect(cv_image)

        # P = np.matrix(self.K)*np.matrix(self.Rt)
        # R = np.matrix(self.Rt[0:3,0:3])
        # RT = R.transpose()


        for kp in keypoints:
            x = int(kp.pt[1])
            y = int(kp.pt[0])

            timestamp = int(time.time())

            # if (timestamp - self.timestamp) < 5000 and \
            #         abs(self.game.inputIRX - x) < 10 and \
            #         abs(self.game.inputIRY - y) < 10:
            #     print "input discarded"
            #     continue
            # self.timestamp = timestamp




            # pass input to the game
            #print str(x) + "  " + str(y)
            #self.game.inputIRX = x
            #self.game.inputIRY = y
            #self.game.inputIRReady = True
            #glutPostRedisplay()

            # RT matrix of RGB camera, use this one for now
            # # TODO: add IR camera offset
            # Rt = np.matrix(self.Rt)
            #
            # # P = K[R|t] is the camera matrix of IR camera
            # P = np.matrix(self.K)*np.matrix(Rt)
            #
            # # get pseudoinverse of Pir for backprojection
            # #PirInv = cv2.invert(Pir)
            #
            # new_transform = np.ndarray(P.shape, np.float32)
            # new_transform[:] = P[:]
            # P_conv = new_transform
            # retval, PInv = cv2.invert(P_conv, flags=cv2.DECOMP_SVD)
            # #PInv = np.linalg.pinv(P)
            # PInv = np.matrix(PInv)
            #
            # PX = np.matrix(PInv)*np.matrix(pt)
            # PX = PX[:]/PX[3]
            #
            # direc = PX[0:3] - C
            # intersectionplane = 2
            # t2 = -C.item(intersectionplane)/direc.item(intersectionplane)
            # planeX = dPoint.item(0)
            # planeY = dPoint.item(1)


            # PirInv = np.linalg.pinv(Pir)
            #
            # # detected point in pixel coords
            # pt = np.matrix([[x], [y], [1]])
            #
            # # back-project detected point
            # PX = np.matrix(PirInv)*np.matrix(pt)
            # # unhomogenize
            # PX = PX[0:3]/PX[3]
            #
            # # get center of the camera in board (world space)
            # C = Rt * np.matrix([[0], [0], [0], [1]])
            #
            # # ray has to pass through between C and PX
            # ray = (C - PX)
            # # to non-homogeneous
            # ray = ray[0:3]
            # # normalize the ray
            # ray = ray / np.linalg.norm(ray)
            #
            # # intersection with xy plane
            # t = C[2]/ray[2]
            #
            # # find out coordinates on the xy-plane
            # planeX = (t*ray[0] + C[0]).item(0)
            # planeY = (t*ray[1] + C[1]).item(0)

            # is input valid?
            # print "---"
            # if planeX > 5 or planeX < -5:
            #     print "wrong x"
            #     print str(planeX) + " " + str(planeY)
            #     print Rt
            #     break
            # if planeY > 5 or planeY < -5:
            #     print "wrong y"
            #     print str(planeX) + " " + str(planeY)
            #     print Rt
            #     break
            # if (timestamp - self.timestamp) < 8000 and \
            #         abs(self.game.inputIRX - planeX) < 0.1 and \
            #         abs(self.game.inputIRY - planeY) < 0.1:
            #     print "input discarded"
            #     continue
            # self.timestamp = timestamp
            #
            # # pass input to the game
            # #print str(x) + "  " + str(y)
            # self.game.inputIRX = planeX
            # self.game.inputIRY = planeY
            # self.game.inputIRReady = True
            # glutPostRedisplay()
            # print Rt
            # print planeX
            # print planeY
            # self.f.write( str(planeX) + ' ' + str(planeY) + "\n")

            break


        # print keypoints
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        #resized = cv2.resize(im_with_keypoints, (0,0), fx=10, fy=10)
        cv2.resizeWindow("Keypoints",640,480)
        cv2.imshow("Keypoints", im_with_keypoints)

        cv2.waitKey(3)


    def callback_pose(self, data):
        if not self.game.ready:
            return

        self.loadCamera()
        glutPostRedisplay()

    def callback_modelview(self, data):
        if not self.game.ready:
            return
        self.loadCamera()
        self.game.modelview = np.array(data.data).flatten() # array(data.data, dtype=float)
        glutPostRedisplay()
        #print "Modelview: "
        #print self.game.modelview[-4:-1]

    def callback_position(self, data):
        print "incoming position!"

    def callback_transform(self, data):
        print "incoming transform!"

    def callback_Rt(self, data):
        #print "Rt received!"
        if not self.game.ready:
            return
        try:
            Rt = self.bridge.imgmsg_to_cv2(data)

        except CvBridgeError, e:
            print e
            return
        self.Rt = np.matrix(Rt.copy())
        self.Pcv = self.K*self.Rt
        #outfile = TemporaryFile()
        np.save("/home/radek/catkin_ws/Rt.npy", self.Rt)
        #C = Rt * np.matrix([[0], [0], [0], [1]])
        #print C.transpose()
        #print self.Rt

    def callback_K(self, data):
        if self.K is not None:
            return
        try:
            K = self.bridge.imgmsg_to_cv2(data)

        except CvBridgeError, e:
            print e
            return
        print "GOT K!"
        self.K = np.matrix(K.copy())
        np.save("/home/radek/catkin_ws/K.npy", self.K)
        #print self.Rt



    def callback_rgb(self, data):
        if self.game.ready:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

                cvHeight, cvWidth, _ = cv_image.shape
                cv2.imwrite("/home/radek/catkin_ws/image.png", cv_image)
                #print str(self.game.height) + ", " + str(self.game.width)
                #print str(cvHeight) + ", " + str(cvWidth)

                #newX = float(self.game.width) / float(cvWidth)
                #newY = float(self.game.height) / float(cvHeight)
                #print "scaleX = " + str(newX)
                #print "scaleY = " + str(newY)
                #resizedFrame =, fx=newX, fy=newY)
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
                cv2.waitKey(1)

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
