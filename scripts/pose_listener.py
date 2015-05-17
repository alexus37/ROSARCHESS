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
import time

from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

# noinspection PyPep8Naming
class pose_listener:
    def __init__(self):
        # intialize python aruco wrapper
        cam.init()

        self.display_stuff = 0
        if self.display_stuff:
            cv2.namedWindow("RGB window", 1)
            cv2.namedWindow("DEPTH window", 1)


        # initialize CV bridge
        self.bridge = CvBridge()

        # initializie listeners
        #self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback_rgb)
        #self.sub_occlusion = rospy.Subscriber('/ar_single_board/occlusion_mask', Image, self.callback_occlusion)
        #self.sub_depth = rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)
        #self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)
        #self.sub_modelview = rospy.Subscriber('/ar_single_board/modelview', Float32MultiArray, self.callback_modelview)
        #self.sub_cam_info = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.callback_cam)
        self.sub_ir = rospy.Subscriber('/IR_data', Image, self.callback_ir)
        self.sub_Rt = rospy.Subscriber('/ar_single_board/Rt', Image, self.callback_Rt)

        # initialize rest of the stuff
        self.timestamp = 0
        self.currentFrame = None
        self.Rt = None
        self.P = None
        self.imX = 640
        self.imY = 480

        self.inputIRX = None
        self.inputIRY = None

        self.x1 = None
        self.x2 = None
        self.x3 = None
        self.x4 = None
        self.y1 = None
        self.y2 = None
        self.y3 = None
        self.y4 = None


        # file for debug logs
        self.f = open('test.txt', 'w')



    def callback_ir(self,data):
        # print "IR image arrived"
        if not self.game.ready:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            self.currentFrame = cv_image
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
        K = np.matrix([[757.164773, 0, 335.313573], [0, 823.419897, 138.458047], [0, 0, 1]])

        # IR CAMERA DISTORTION COEFFICIENTS
        #distortion = np.matrix([-0.321378, -0.018411, -0.012105, -0.003352, 0])

        # UNDISTORT THE IR IMAGE - with current calibration it only makes it worse
        # cv_image = cv2.undistort(cv_image, K, distortion)

        # BLOB DETECTION
        keypoints = detector.detect(cv_image)

        for kp in keypoints:
            x = int(kp.pt[1])
            y = int(kp.pt[0])

            timestamp = int(time.time())


            # RT matrix of RGB camera, use this one for now
            # TODO: add IR camera offset
            Rt = np.matrix(self.Rt)

            # P = K[R|t] is the camera matrix of IR camera
            Pir = np.matrix(K)*np.matrix(Rt)

            # get pseudoinverse of Pir for backprojection
            #PirInv = cv2.invert(Pir)
            PirInv = np.linalg.pinv(Pir)

            # detected point in pixel coords
            pt = np.matrix([[x], [y], [1]])

            # back-project detected point
            PX = np.matrix(PirInv)*np.matrix(pt)
            # unhomogenize
            PX = PX[0:3]/PX[3]

            # get center of the camera in board (world space)
            C_rgb = Rt * np.matrix([[0], [0], [0], [1]])

            # ray has to pass through between C and PX
            ray = (C_rgb - PX)
            # to non-homogeneous
            ray = ray[0:3]
            # normalize the ray
            ray = ray / np.linalg.norm(ray)

            # intersection with xy plane
            t = C_rgb[2]/ray[2]

            # find out coordinates on the xy-plane
            planeX = (t*ray[0] + C_rgb[0]).item(0)
            planeY = (t*ray[1] + C_rgb[1]).item(0)

            # is input valid?
            print "---"
            if planeX > 5 or planeX < -5:
                print "wrong x"
                print str(planeX) + " " + str(planeY)
                print Rt
                break
            if planeY > 5 or planeY < -5:
                print "wrong y"
                print str(planeX) + " " + str(planeY)
                print Rt
                break
            if (timestamp - self.timestamp) < 8000 and \
                    abs(self.inputIRX - planeX) < 0.1 and \
                    abs(self.inputIRY - planeY) < 0.1:
                print "input discarded"
                continue
            self.timestamp = timestamp

            # pass input to the game
            #print str(x) + "  " + str(y)
            self.inputIRX = planeX
            self.inputIRY = planeY
            print Rt
            print planeX
            print planeY
            self.f.write( str(planeX) + ', ' + str(planeY) + "\n")

            if self.x1 is None:
                self.x1 = planeX
                self.y1 = planeY
                print "pt1 set"
            if self.x2 is None:
                self.x2 = planeX
                self.y2 = planeY
                print "pt2 set"
            if self.x3 is None:
                self.x3 = planeX
                self.y3 = planeY
                print "pt3 set"
            if self.x4 is None:
                self.x4 = planeX
                self.y4 = planeY
                print "pt4 set"
            break

        if self.x1 is not None and self.x2 is not None and self.x3 is not None and self.x4 is not None and \
            self.y1 is not None and self.y2 is not None and self.y3 is not None and self.y4 is not None:
            objectPoints = np.array([[-0.1, -0.1, 0],
                                     [0.1, -0.1, 0],
                                     [0.1, 0.1, 0],
                                     [-0.1, 0.1, 0]
                                     ])
            imagePoints = np.array([[self.x1, self.y1],
                                    [self.x2, self.y2],
                                    [self.x3, self.y3],
                                    [self.x4, self.y4]
                                    ])

            cameraMatrix = K

            distCoeffs =  np.zeros((5,1))

            retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
            rot = np.matrix(cv2.Rodrigues(rvec))
            tvec = np.matrix(cv2.Rodrigues(tvec))

            RtIr = np.concatenate((rot, tvec), axis=1)

            C_ir = RtIr * np.matrix([[0], [0], [0], [1]])

            print "rgb: "
            print C_rgb
            print "ir: "
            print C_ir


        # print keypoints
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        #resized = cv2.resize(im_with_keypoints, (0,0), fx=10, fy=10)
        cv2.resizeWindow("Keypoints",640,480)
        cv2.imshow("Keypoints", im_with_keypoints)

        cv2.waitKey(3)


    def callback_Rt(self, data):
            #print "Rt received!"
            if not self.game.ready:
                return
            try:
                Rt = self.bridge.imgmsg_to_cv2(data)

            except CvBridgeError, e:
                print e
                return
            self.Rt = Rt.copy()
            #print self.Rt

def listener():
    pl = pose_listener()
    rospy.init_node('pose_listener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
