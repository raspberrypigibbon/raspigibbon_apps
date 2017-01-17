#!/usr/bin/env python
# coding: utf-8

from raspigibbon_utils import read_eef
import rospy
from sensor_msgs.msg import JointState

class Master:
	def __init__(self):
		self.rs = read_eef.RS304MD()
		self.pub = rospy.Publisher("master_joint_state", JointState, queue_size=10)
		for i in range(1,6):
			self.rs.setTorque(i, False)
			rospy.sleep(0.01)
		print "init done"


	def run(self):
		while not rospy.is_shutdown():
			js = JointState()
			js.name=["joint{}".format(i) for i in range(1,6)]
			js.position = [max(-150,min(150,self.rs.readAngle(i))) for i in range(1,6)]
			self.pub.publish(js)
			rospy.sleep(0.01)

if __name__ == "__main__":
	rospy.init_node("master_joint_state")
	master = Master()
	master.run()
