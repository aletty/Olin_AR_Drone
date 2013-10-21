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
  def __init__(self, droneController,topicName='/joy'):
    self.topicName = topicName
    self.droneController = droneController
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

    self.scale = {
      "ScaleRoll": 1.0,
      "ScalePitch": 1.0,
      "ScaleYaw": 1.0,
      "ScaleZ": 1.0
    }

    self.subJoyData = rospy.Subscriber(self.topicName, Joy, self.ReceiveJoy)
    self.unblock = False
    

  def ReceiveJoy(self, data):
    if not any(button != 0 for button in data.buttons):
      self.ProcessJoy(data)
    elif not self.unblock:
      self.unblock = any(button != 0 for button in data.buttons)
      self.ProcessJoy(data)

  def ProcessJoy(self, data):
    if data.buttons[self.buttons['ButtonEmergency']]==1:
      rospy.loginfo("Emergency Button Pressed")
      self.droneController.SendEmergency()
      self.UnblockButton()

    elif data.buttons[self.buttons['ButtonLand']]==1:
      rospy.loginfo("Land Button Pressed")
      self.droneController.SendLand()
      self.UnblockButton()
    
    elif data.buttons[self.buttons['ButtonTakeoff']]==1:
      rospy.loginfo("Takeoff Button Pressed")
      self.droneController.SendTakeoff()
      self.UnblockButton()
    
    elif data.buttons[self.buttons['ButtonFlattrim']]==1:
      rospy.loginfo("Flattrim Button Pressed")
      self.droneController.CallFlattrim()
      self.UnblockButton()
    
    elif data.buttons[self.buttons['ButtonHover']]==1:
      rospy.loginfo("Hover Button Pressed")
      self.droneController.SendHover()
      self.UnblockButton()
    
    else:
      self.droneController.SetCommand(data.axes[self.axes['AxisRoll']]/self.scale['ScaleRoll'],data.axes[self.axes['AxisPitch']]/self.scale['ScalePitch'],data.axes[self.axes['AxisYaw']]/self.scale['ScaleYaw'],data.axes[self.axes['AxisZ']]/self.scale['ScaleZ'])
    
  def UnblockButton(self):
    self.unblock = False

# Setup the application
if __name__=='__main__':
  import sys
  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('ardrone__new_joystick_controller')

  # Now we construct our Qt Application and associated controllers and windows
  app = QtGui.QApplication(sys.argv)
  display = DroneVideoDisplay()

  controller = BasicDroneController()

  joy = JoystickController(controller)

  # executes the QT application
  display.show()
  status = app.exec_()

  # and only progresses to here once the application has been shutdown
  rospy.signal_shutdown('Great Flying!')
  sys.exit(status)