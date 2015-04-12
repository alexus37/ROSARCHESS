#!/usr/bin/env python
from OpenGL.raw.GLUT import glutPostRedisplay
import rospy
import cv2
import numpy as np
import tf 
import sys 
import os
import threading

sys.path.insert(0, "/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
import GameNoLogic


from tf.transformations import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

# noinspection PyPep8Naming
class kinect_listener:

  def __init__(self):
   # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.display_stuff = 0
    if self.display_stuff:
        cv2.namedWindow("RGB window", 1)
        cv2.namedWindow("DEPTH window", 1)

    self.game = GameNoLogic.Game()

    self.bridge = CvBridge()
    self.currentFrame = 0
    self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color',Image, self.callback_rgb)
    self.sub_depth = rospy.Subscriber('/camera/depth/image', Image, self.callback_depth)
    self.sub_pose = rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.callback_pose)

    # self.sub_transform = rospy.Subscriber('/ar_single_board/transform', TransformStamped , self.callback_transform)
    # self.sub_position = rospy.Subscriber('/ar_single_board/position', Vector3Stamped , self.callback_position)

    # DO EVERYTHING BEFORE THIS POINT!!!
    os.chdir( "/home/radek/3dPhoto/AugmentedRealityChess/pythonAnimations/pyOpenGLChess/")
    # thread.start_new_thread(self.start_game,())
    # self.game.start()
    thread = threading.Thread(target=self.start_game, args=())
    thread.daemon = False
    thread.start()

  def callback_pose(self,data):
    q = data.pose.orientation
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    matrix = euler_matrix(euler[0],euler[1],euler[2])
    pos = data.pose.position
    pos = [[-pos.x], [-pos.y], [-pos.z], [1]]
    pos = np.matrix(pos)
   # print "========="
    #print np.transpose(pos)
   # print np.matrix(euler)*180/np.pi

    #matrix[0][3] = -pos.x
    #matrix[1][3] = -pos.y
    #matrix[2][3] = -pos.z
    inv = np.matrix(inverse_matrix(matrix))

    worldPos = inv*pos

    #cam_or = np.array([[0], [0], [0], [1]])
    #cam_or = np.dot(matrix, cam_or)
    #print matrix
    #print cam_or
    #cam_or[0:3] = cam_or[0:3]/cam_or[3]

    #print np.transpose(worldPos)

    up = np.matrix([[0], [1], [0], [0]])
    trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],  [pos[0], pos[1], pos[2], 1]])
    upTrans = inv*trans*up

    newCamPos = 10*worldPos[0:3]
    newCamUp = -upTrans[0:3]

    delta = abs((newCamPos-self.game.camPos).sum())
    print delta

    if delta> 0.01:

        self.game.camPos = newCamPos
        self.game.up = newCamUp
        # self.game.redraw()
        glutPostRedisplay()

    #print cam_or[0:2]

  def callback_position(self,data):
    print "incoming position!"

  def callback_transform(self,data):
    print "incoming transform!"


  def callback_rgb(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.currentFrame = cv_image;
    except CvBridgeError, e:
        print e
    if self.display_stuff:
        cv2.imshow("RGB window", cv_image)
        cv2.waitKey(3)
    
  def callback_depth(self,data):
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
