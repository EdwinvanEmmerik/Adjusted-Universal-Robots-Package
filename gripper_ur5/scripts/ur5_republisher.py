#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class ur5_republisher():
## Class ur5_republisher
## Description:
## The class will initialize with an subscriber on the topic '/joint_states_ur5'. Everytime the
## ur_modern_driver will publish the joint_states under the topic '/joint_states' the message gets
## remapped to '/joint_states_ur5' in the ur_common.launch launchfile. When the subscriber gets triggered
## the joint_state_callback will be triggered and this will add the dynamixel_gear_joint JointState to the
## message as a tuple.
	def __init__(self):
		rospy.init_node('Cmd_Scaler')
		self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
		rospy.Subscriber("/joint_states_ur5", JointState, self.joint_state_callback)
		self.name = 'dynamixel_gear_joint'
		self.position = self.checkGripperState()
		self.velocity = 0
		self.effort = 0

	def appendT(self,t,x):
		l = list(t)
		l.append(x)
		return tuple(l)

	def checkGripperState(self):
		try:
			gripper_state = rospy.get_param('/bool_gripperOpen_param')
			if(gripper_state):
				gripper_position = 0.25
				return gripper_position
			if not(gripper_state):
				gripper_position = 0.0
				return gripper_position
		except KeyError:
			gripper_state = 0.0
			print("rosparam, bool_gripperOpen_param not set")
			return gripper_state

	def joint_state_callback(self, data):
		data.name.append(self.name)
		data.position = self.appendT(data.position, self.position)
		data.velocity = self.appendT(data.velocity, self.position)
		data.effort = self.appendT(data.effort, self.position)
		self.pub.publish(data)

if __name__ == '__main__':
	ur5_r = ur5_republisher()
	while not rospy.is_shutdown():
		# self.cycle()
		rospy.spin()
"""
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class ur5_republisher():
	def __init__(self):
		rospy.init_node('Cmd_Scaler')
		self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
		rospy.Subscriber("/joint_states_ur5", JointState, self.joint_state_callback)
		#rospy.Subscriber("/GripperState", Bool, self.gripper_state_callback)
		self.name = 'dynamixel_gear_joint'
		#self.gripper_state = rospy.get_param('/bool_gripperOpen_param', False)
		#if self.gripper_state:
		self.position = 0.1
		#if not(self.gripper_state):
		#    self.position = 0
		self.velocity = 0
		self.effort = 0

	def appendT(self,t,x):
		l = list(t)
		l.append(x)
		return tuple(l)

	def joint_state_callback(self, data):
		data.name.append(self.name)
		data.position = self.appendT(data.position, self.position)
		data.velocity = self.appendT(data.velocity, self.position)
		data.effort = self.appendT(data.effort, self.position)
		self.pub.publish(data)

if __name__ == '__main__':
	ur5_r = ur5_republisher()
	while not rospy.is_shutdown():
		# self.cycle()
		rospy.spin()
"""