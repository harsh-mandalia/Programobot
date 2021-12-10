#!/usr/bin/env python

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Quaternion 
import rospy
import tf
import numpy as np

class sbb():
	def __init__(self):

		rospy.init_node('controller')

		self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.sbb_orientation_euler = [0.0, 0.0, 0.0]
		self.angular_vel=[0, 0, 0]

		self.sample_rate = 50.0  # in Hz

		self.i = 0.0
		self.curr_angle = 0.0
		self.prev_angle = 0.0
		self.reset = 0

		self.kp=500
		self.kd=0
		self.a=0
		
		self.flywheel_cmd = Float32()
		self.flywheel_cmd.data = 0.0

		self.stand1_cmd=Float32()
		self.stand1_cmd.data=0

		self.stand2_cmd=Float32()
		self.stand2_cmd.data=0

		self.handle_cmd=Float32()
		self.handle_cmd.data=0

		self.drive_cmd=Float32()
		self.drive_cmd.data=0

		rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
		self.flywheel_vel=0
		rospy.Subscriber('/flywheel/joint_state', JointState, self.vel_callback)

		self.flywheel_pub = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
		self.stand1_pub = rospy.Publisher('/stand1/command', Float32, queue_size=1)
		self.stand2_pub = rospy.Publisher('/stand2/command', Float32, queue_size=1)
		self.handle_pub = rospy.Publisher('/handle/command', Float32, queue_size=1)
		self.drive_pub = rospy.Publisher('/drive_wheel/command', Float32, queue_size=1)



	def imu_callback(self, msg):
		# print("imu")
		self.sbb_orientation_quaternion[0] = msg.orientation.x
		self.sbb_orientation_quaternion[1] = msg.orientation.y
		self.sbb_orientation_quaternion[2] = msg.orientation.z
		self.sbb_orientation_quaternion[3] = msg.orientation.w
		self.angular_vel[0]=msg.angular_velocity.x

	def vel_callback(self, msg):
		# print("hi")
		self.flywheel_vel=msg.velocity[0]

	def temppid(self):
		(self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
		print(np.rad2deg(self.sbb_orientation_euler[1]))
		self.curr_angle = self.sbb_orientation_euler[1]
	    # print(self.angular_vel[0])

		while(True):
			temp=int(input())
			self.a+=temp
			self.data_cmd.data=self.a
			self.data_pub.publish(self.data_cmd)
			if(temp==314):
				break

	def zeroAll(self):
		# if(abs(self.flywheel_vel)<0.2):
		# 	exit()
		if(abs(self.flywheel_vel)>2):
			if(self.flywheel_vel>0):
				self.flywheel_cmd.data=self.flywheel_vel-1
			if(self.flywheel_vel<0):
				self.flywheel_cmd.data=self.flywheel_vel+1
		else:
			self.flywheel_cmd.data=self.flywheel_vel/2
		print(self.flywheel_vel, self.flywheel_cmd.data)

		self.stand1_cmd.data=0
		self.stand2_cmd.data=0
		self.handle_cmd.data=0
		
		self.stand1_pub.publish(self.stand1_cmd)
		self.stand2_pub.publish(self.stand2_cmd)
		self.handle_pub.publish(self.handle_cmd)
		self.flywheel_pub.publish(self.flywheel_cmd)
		self.drive_pub.publish(self.drive_cmd)

            

	    




if __name__ == '__main__':

	sbb = sbb()
	r = rospy.Rate(sbb.sample_rate)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		try:    
			# sbb.temppid()
			sbb.zeroAll()
			rospy.sleep(0.1)
		except rospy.exceptions.ROSTimeMovedBackwardsException: pass