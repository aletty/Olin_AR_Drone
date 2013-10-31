#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_vision')

import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/ardrone/image_cv",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)

  def ReceiveImage(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
    except CvBridgeError, e:
      print e

    try:
      self.image_pub.publish(cv_image)
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down converter"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)