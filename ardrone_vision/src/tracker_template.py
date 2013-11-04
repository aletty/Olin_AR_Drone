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

    self.template = cv2.imread("/home/arjun/ros/sandbox/Olin_AR_Drone/ardrone_vision/src/fiducial_template.png")
    print("TYPE of template", type(self.template))
    self.match_method = cv2.TM_CCOEFF
    self.template_w, self.template_h = self.template.shape[]

    self.tracked_object = Point()

  def ConvertImage(self, data):
    try:
      cv_mat = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.img = np.asarray(cv_mat[:,:])
      self.ProcessImage()
    except CvBridgeError, e:
      print e    

  def ProcessImage(self):
    """Performs template matching on source image to find fiducial"""

    # apply template matching
    res = cv2.matchTemplate(self.img, self.template, self.match_method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    
    # rectangle with mathced fiducial
    top_left = max_loc
    bottom_right = (top_left[0]+self.template_w, top_left[1]+self.template_h)

    self.Draw(top_left, bottom_right)
    # cv2.imshow(self.color, thresh)
    # cv2.imshow("HSV", hsv_img)
    #print circles

  def Draw(self,top_left, bottom_right):
    # Drawing Rectange
    print("Top left of rectangle: ", top_left)
    print("Bottom right of rectangle: ", bottom_right)
    cv2.rectangle(self.img, top_left, bottom_right, cv.RGB(0,0,255), 2)
    cv2.imshow("template matching", self.img)

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