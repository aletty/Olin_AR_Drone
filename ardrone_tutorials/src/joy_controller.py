#!/usr/bin/env python

# Updated Joystick controller

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Import the joystick message
from sensor_msgs.msg import Joy

# Finally the GUI libraries
from PySide import QtCore, QtGui

class JoystickController(object):
  def __init__(self, topicName='/joy'):
    self.topicName = topicName
    
    self.buttons = {
      "ButtonEmergency": 8,
      "ButtonLand": 1,
      "ButtonTakeoff": 0,
      "ButtonFlattrim": 7,
      "ButtonHover": 2
    }
    
    self.axes = {
      "AxisRoll": 0,
      "AxisPitch": 1,
      "AxisYaw": 3,
      "AxisZ": 4
    }

    self.subJoyData = rospy.Subscriber(self.topic_name, Joy, self.ReceiveJoy)
    self.debounce = False
    

  def ReceiveJoy(self, data):
    if self.debounce:
      pass
    else:
      self.debounce = any(button != 0 for button in data.buttons)
      self.ProcessJoy(data)

  def ProcessJoy(self, data):
    if data.buttons[self.buttons['ButtonEmergency']]==1:
      rospy.loginfo("Emergency Button Pressed")
      controller.SendEmergency()
    elif data.buttons[ButtonLand]==1:
      rospy.loginfo("Land Button Pressed")
      controller.SendLand()
    elif data.buttons[ButtonTakeoff]==1:
      rospy.loginfo("Takeoff Button Pressed")
      controller.SendTakeoff()
    elif data.buttons[ButtonFlattrim]==1:
      rospy.loginfo("Flattrim Button Pressed")
      controller.CallFlattrim()
    elif data.buttons[ButtonHover]==1:


class Joystick(object):
    def __init__(self, topic_name='/joy'):
        self.topic_name = topic_name
        self.buttons_names = {'l1':10, 'l2':8}
        self.sub = rospy.Subscriber(topic_name, Joy, self.cb)
        self.blocked = False
        self.sched = sched.scheduler(time.time, time.sleep)
        print '>>> press L1 button'

    def cb(self, data):
        print '>> in callback:', data.buttons[self.buttons_names['l1']], data.buttons[self.buttons_names['l2']]
        if not self.blocked:
            self.foo(data)

    def foo(self, data):
        print '>> in foo:', data.buttons[self.buttons_names['l1']], data.buttons[self.buttons_names['l2']]
        if data.buttons[self.buttons_names['l1']]:
            self.blocked = True
            rospy.loginfo('%d', data.buttons[self.buttons_names['l1']])
            rospy.loginfo('L1 pressed')
            self.sched.enter(1, 1, self.bar, ())
        self.sched.run()

    def bar(self):
        rospy.loginfo('resuming control')
        self.blocked = False

if __name__=='__main__':
    rospy.init_node('foo', anonymous=False)
    joy = Joystick()
    rospy.spin()