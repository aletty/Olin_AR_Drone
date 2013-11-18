#! /usr/bin/env python

# ROS libraries
import roslib; roslib.load_manifest('ardrone_vision')
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from drone_controller import BasicDroneController

# Other libraries
import sys
from threading import Thread
import cv2
import numpy as np
import yaml

class Tracker(Thread):
  def __init__(self, controller, cam_calib = '/home/arjun/ros/sandbox/Olin_AR_Drone/ardrone_vision/src/cal.yaml', template = None):
    Thread.__init__(self)

    # drone controller
    self.controller = controller
    
    # ROS publisher and subscriber setup
    self.cv_object_pub = rospy.Publisher('/ardrone/object_tracker', Point)

    # ROS CV2 conversion 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/ardrone/bottom/image_raw',Image,self.ConvertImage, queue_size=None)

    # initialize CV2 constants
    self.LoadCalib(cam_calib)
    self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    self.objp = np.zeros((8*6,3), np.float32)
    self.objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    self.axis = np.float32(([[3,0,0],[0,3,0],[3,3,0],[3,0,0]])).reshape(-1,3)
    
    # tracked object is type point
    self.tracked_object = Point()

  def LoadCalib(self, cam_calib):
    """opens camera calibration file and retrieve
    relevant data"""
    with open(cam_calib, 'r') as f:
      calib_dict = yaml.safe_load(f)
    self.cam_mtx = np.reshape(calib_dict['camera_matrix']['data'], (3,3))
    self.dist_mtx = np.array(calib_dict['distortion_coefficients']['data'])

  def ConvertImage(self, data):
    try:
      cv_mat = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.img = np.asarray(cv_mat[:,:])
      self.ProcessImage()
    except CvBridgeError, e:
      print e    

  def ProcessImage(self):
    gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray_img, (8,6), None, flags = cv2.CALIB_CB_FAST_CHECK )
    if ret == True:
      rvecs, tvecs, inliers = cv2.solvePnPRansac(self.objp, corners, self.cam_mtx, self.dist_mtx)
      self.imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.cam_mtx, self.dist_mtx)
      print "Rotation: ", rvecs
      print "Translation: ", tvecs
      gray_img = self.Draw(corners, gray_img)
      cv2.imshow('drone', gray_img)
      k = cv2.waitKey(0) & 0xff
    else:
      cv2.imshow('drone', gray_img)
    
    if cv2.waitKey(1) >= 0:
      return

  def Draw(self,corners, gray_img):
    self.imgpts = np.int32(self.imgpts).reshape(-1,2)
    gray_img = cv2.drawContours(gray_img, [self.imgpts[:4]], -1, (0,255,0), 3)
    print "Gray image type: ", type(gray_img)
    # publish instead of returning
    # self.cv_object_pub.publish(self.tracked_object)


if __name__ == '__main__':
  print "Starting Drone Tracker:"

  cont = BasicDroneController()
  track = Tracker(cont)
  track.start()

  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('object_tracker')

  try:
    rospy.spin()
  except KeyboardInterrupt, SystemExit:
    print "Shutting down Tracker"