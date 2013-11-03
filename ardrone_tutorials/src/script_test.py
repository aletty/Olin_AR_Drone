#!/usr/bin/env python

# The Joystick Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Joystick Control"
# https://github.com/mikehamer/ardrone_tutorials

# This controller implements the base DroneVideoDisplay class, the DroneController class and subscribes to joystick messages

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Import the joystick message
from sensor_msgs.msg import Joy
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# Finally the GUI libraries
from PySide import QtCore, QtGui

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 8
ButtonLand      = 1
ButtonTakeoff   = 0
ButtonFlattrim  = 7
ButtonHover     = 2

# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

button_last_state = 0
az_acc = 0


def RunScript():

  rospy.loginfo("Starting Script")
  controller.CallFlattrim()
  rospy.sleep(1.0)
  rospy.loginfo("Takeing Off")
  controller.SendTakeoff()
  rospy.loginfo("Hovering")
  controller.SendHover()
  rospy.sleep(6.0)
  while az_acc > .7:
  	rospy.loginfo(az_acc)
  rospy.sleep(2.0)

  # controller.SetCommand(pitch=3)
  # rospy.sleep(0.3)
  # controller.SetCommand(roll=3)
  # rospy.sleep(0.3)
  # controller.SetCommand(pitch=-3)
  # rospy.sleep(0.3)	
  # controller.SetCommand(roll=-3)
  # rospy.sleep(0.3)

  # rospy.loginfo("Forward")
  # controller.SetCommand(pitch=2)
  # rospy.sleep(1.0)
  # rospy.loginfo("Hovering")
  # controller.SendHover()
  # rospy.sleep(1.0)
  # rospy.loginfo("Back")
  # controller.SetCommand(pitch=-2)
  # rospy.sleep(1.0)
  # rospy.loginfo("Hovering")

  controller.SendHover()
  rospy.sleep(1.0)
  controller.SendLand()
  rospy.loginfo("Landed")

def ReceiveNavdata(navinfo):
	global az_acc
	az_acc = navinfo.az


# handles the reception of joystick packets
def ReceiveJoystickMessage(data):
	global button_last_state

	if data.buttons[ButtonEmergency]==1:
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
		#rospy.loginfo("Hover Button Pressed")
		#controller.SendHover()
		if button_last_state == 0:
			rospy.loginfo("SCRIPT Button Pressed")
			RunScript()
			button_last_state = 1
		else:
			rospy.loginfo("SCRIPT Passed")
	else:
		if button_last_state == 1:
			button_last_state = 0
			rospy.loginfo("SCRIPT Toggle Reset")
		else:
			pass
		controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('script_controller')

	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	ButtonFlattrim  = int (		rospy.get_param("~ButtonFlattrim", ButtonFlattrim) )
	ButtonHover			= int (		rospy.get_param("~ButtonHover", ButtonHover) )
	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )


	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	controller = BasicDroneController()

	# subscribe to the /joy topic and handle messages of type Joy with the function ReceiveJoystickMessage
	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)
	subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveNavdata) 
	
	# executes the QT application
	display.show()
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)