#! /usr/bin/env python

# ROS libraries
import roslib; roslib.load_manifest('ardrone_vision')
import rospy
from std_msgs.msg import Empty as msgEmpty         # for land/takeoff/emergency

import sys
from math import sqrt
from threading import Thread
import cv
import cv2
import numpy as np

# Global Variables
VideoCapture = cv2.VideoCapture(0)
circles = (0,0,0)

COLOR_RANGE={
'yellow': ((10, 100, 100), (40, 255, 255)),\
'red': ((170, 80, 0), (190, 255, 255)),\
'blue': (( 70 , 31 , 11), ( 120 , 255 , 255)),\
'purple': (( 170 , 5 , 40), (180 , 255 , 255)),\
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
  def __init__(self, color, flag):
    Thread.__init__(self)
    self.color=color
    self.display=DISPLAY_COLOR[color]
    self.path=np.zeros((480,640,3), dtype=np.uint8)
    self.lastx=0
    self.lasty=0
    self.h_min=COLOR_RANGE[color][0]
    self.h_max=COLOR_RANGE[color][1]
    self.flag=flag
    if self.flag:
      cv2.namedWindow(self.color,1)

  def poll(self, img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv_img, self.h_min, self.h_max)
    thresh = cv2.GaussianBlur(thresh, (13,13), 0)
    thresh = cv2.GaussianBlur(thresh, (13,13), 0)
    circles = cv2.HoughCircles((thresh), cv.CV_HOUGH_GRADIENT,dp=2,minDist=400,minRadius=15, maxRadius=200)
    cv2.imshow(self.color, thresh)
    cv2.imshow("HSV", hsv_img)
    #print circles
    self.draw(thresh, circles)

  def draw(self,thresh, circles):
    # Drawing Things
    maxRadius = 0
    x = 0
    y = 0
    found = False

    if circles is not None:
      for circle in circles[0]:
        if circle[2] > maxRadius:
          found = True
          radius = int(circle[2])
          maxRadius = int(radius)
          x = int(circle[0])
          y = int(circle[1])

      if found:
        cv2.circle(img, (x,y), 3, self.display, -1, 8, 0)
        cv2.circle(img, (x,y), maxRadius, self.display, 3, 8, 0)
        return {'image': img, 'target': (x,y)}
        # self.pubLand.publish(msgEmpty())
        #print self.color + " ball found at: (", x, ",", y, ")"

    if self.flag:
      cv2.imshow(self.color, thresh)

    # cv2.imshow("result", img)
    if cv2.waitKey(1) >= 0:
      return

if __name__ == '__main__':
  print "Starting MultipleColorTracker:"

  cv2.namedWindow("Result", 0)
  if VideoCapture:
    frame_copy = None
  # yellow = Tracker("yellow", True)
  # green = Tracker("green", True)
  # blue = Tracker("blue", True)
  # red = Tracker("red", True)
  # green.start()
  # blue.start()
  # red.start()
  purple = Tracker("purple", True)
  purple.start()

  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('ardrone_vision_controller')

  while not rospy.is_shutdown():
    try:
      success, img = VideoCapture.read() 
      if success:
        #print "Grabbed frame:"
        # blue.poll(img)
        #green.poll(img)
        # blue.join()
        #green.join()
        # green.poll(img)
        # green.join()
        purple.poll(img)
        purple.join()
      else:
        2+2
        #print "Failed to grab new frame."
      if cv2.waitKey(5) != -1:
        break
    except KeyboardInterrupt, SystemExit:
      break