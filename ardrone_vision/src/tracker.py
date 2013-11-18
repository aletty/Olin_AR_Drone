#! /usr/bin/env python

# ROS libraries
import roslib; roslib.load_manifest('ardrone_vision')
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from drone_controller import BasicDroneController

# Other libraries
import sys
from math import sqrt
from threading import Thread
import cv
import cv2
import numpy as np

# Global Variables
circles = (0,0,0)

COLOR_RANGE={
'yellow': ((10, 100, 100), (40, 255, 255)),\
'red': ((170, 80, 0), (190, 255, 255)),\
'blue': (( 70 , 31 , 11), ( 120 , 255 , 255)),\
'purple': (( 100 , 170, 25), (200 , 210, 160)),\
'green': (( 40 , 25 , 60), (50 , 150 , 150)),\
'orange': (( 160 , 100 , 47), (179 , 255 , 255))\
}

DISPLAY_COLOR={
'yellow':cv.RGB(255,255,0)
,'red':cv.RGB(255,0,0)
,'blue':cv.RGB(0,0,255)
,'green':cv.RGB(0,255,0)
,'purple': cv.RGB(255,0,255)
}


class Tracker(Thread):
  def __init__(self, color, controller, flag = False):
    Thread.__init__(self)

    self.controller = controller
    # ROS publisher and subscriber setup
    self.cv_object_pub = rospy.Publisher('/ardrone/object_tracker', Point)

    # ROS CV conversion sub
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/ardrone/image_raw',Image,self.ConvertImage, queue_size=None)

    self.color=color
    self.display=DISPLAY_COLOR[color]
    self.path=np.zeros((480,640,3), dtype=np.uint8)
    self.lastx=0
    self.lasty=0
    self.h_min=COLOR_RANGE[color][0]
    self.h_max=COLOR_RANGE[color][1]
    self.flag=flag

    self.tracked_object = Point()

    if self.flag:
      cv2.namedWindow(self.color,1)

  def ConvertImage(self, data):
    try:
      cv_mat = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.img = np.asarray(cv_mat[:,:])
      self.ProcessImage()
    except CvBridgeError, e:
      print e    

  def ProcessImage(self):
    hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv_img, self.h_min, self.h_max)
    # thresh = cv2.GaussianBlur(thresh, (13,13), 0)
    thresh = cv2.GaussianBlur(thresh, (13,13), 0)
    circles = cv2.HoughCircles((thresh), cv.CV_HOUGH_GRADIENT,dp=2,minDist=200,minRadius=10, maxRadius=400)
    # cv2.imshow(self.color, thresh)
    # cv2.imshow("HSV", hsv_img)
    #print circles
    self.Draw(thresh, circles)

  def Draw(self,thresh, circles):
    # Drawing Circle 
    maxRadius = 0
    self.tracked_object.x = None
    self.tracked_object.y = None
    found = False

    if circles is not None:
      for circle in circles[0]:
        if circle[2] > maxRadius:
          found = True
          radius = int(circle[2])
          # maxRadius = int(radius)
          self.tracked_object.x = int(circle[0])
          self.tracked_object.y = int(circle[1])

      if found:
        cv2.circle(self.img, (self.tracked_object.x,self.tracked_object.y), 3, self.display, -1, 8, 0)
        cv2.circle(self.img, (self.tracked_object.x,self.tracked_object.y), maxRadius, self.display, 3, 8, 0)

        # publish instead of returning
        self.cv_object_pub.publish(self.tracked_object)
        print("radius: ", radius)
        print("Object position: ", self.tracked_object.x, self.tracked_object.y)
        
        # action to the drone
        self.controller.SendHover()
        rospy.sleep(0.1)
        self.controller.SetCommand(z_velocity = -0.5)
        rospy.sleep(0.5)
        self.controller.SendLand()


    if self.flag:
      cv2.imshow(self.color, thresh)
      cv2.imshow("result", self.img)

    if cv2.waitKey(1) >= 0:
      return

if __name__ == '__main__':
  print "Starting Drone Tracker:"

  cont = BasicDroneController()
  purple = Tracker("purple", cont,True)
  purple.start()

  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('object_tracker')

  try:
    rospy.spin()
  except KeyboardInterrupt, SystemExit:
    print "Shutting down Tracker"