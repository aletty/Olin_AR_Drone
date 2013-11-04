#!/usr/bin/env python

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty as msgEmpty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# Import messages for service calls
from std_srvs.srv import Empty as srvEmpty

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	def __init__(self, ns = ''):
		# Holds the current drone status
		self.ns = ns
		self.status = -1

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber(self.ns+'/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher(self.ns + '/ardrone/land',msgEmpty)
		self.pubTakeoff = rospy.Publisher(self.ns + '/ardrone/takeoff',msgEmpty)
		self.pubReset   = rospy.Publisher(self.ns + '/ardrone/reset',msgEmpty)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher(self.ns + '/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def CallFlattrim(self):
		rospy.wait_for_service(self.ns + '/ardrone/flattrim')
		try:
			self.Flattrim = rospy.ServiceProxy(self.ns + '/ardrone/flattrim', srvEmpty)
			self.Flattrim()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def ToggleCam(self):
		rospy.wait_for_service(self.ns + '/ardrone/togglecam')
		try:
			self.camera = rospy.ServiceProxy(self.ns + '/ardrone/togglecam', srvEmpty)
			self.camera()
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state


	def SendHover(self):
		#Send a hover in place message to ardrone drive
		self.SetCommand(0,0,0,0,0,0)

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.SendHover()
			self.pubTakeoff.publish(msgEmpty())
			self.SendHover()

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.SendHover()
		self.pubLand.publish(msgEmpty())
		self.SendHover()

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(msgEmpty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0, ang_x=0, ang_y=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity
		self.command.angular.x = ang_x
		self.command.angular.y = ang_y

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)