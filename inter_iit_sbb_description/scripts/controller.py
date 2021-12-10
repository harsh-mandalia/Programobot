#!/usr/bin/env python

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Quaternion 
from rosgraph_msgs.msg import Clock
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt 
# from scipy.integrate import ode
# from input import getdata
import cv2

class sbb():
	def __init__(self):
		
		rospy.init_node('controller')

		# self.ode = ode(self.func).set_integrator('vode', nsteps=500, method='bdf')
		# self.first=True
		self.time=0
		self.integral=0
		self.t0=0
		self.t1=0

		self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.sbb_orientation_euler = [0.0, 0.0, 0.0]
		self.angular_vel=[0, 0, 0]
		self.stand1_angle=0
		self.stand1_vel=0
		self.stand2_angle=0
		self.stand2_vel=0
		self.handle_angle=0
		self.handle_vel=0

		self.sample_rate = 5.0  # in Hz
		self.i = 0.0
		self.curr_angle = 0.0
		self.prev_angle = 0.0
		self.reset = 0

		self.kp=5
		self.kd=0

		# for plot
		#-----
		self.angles=[]
		self.stand1s=[]
		self.stand2s=[]
		self.handles=[]

		# plt.ion()
		# figure, ax = plt.subplots()
		# line1, = ax.plot(self.angles)
		# plt.figure()
		# plt.title("angles")

		#-----

		self.speed=0
		self.angle=0

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

		self.flywheel_pub = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
		self.stand1_pub = rospy.Publisher('/stand1/command', Float32, queue_size=1)
		self.stand2_pub = rospy.Publisher('/stand2/command', Float32, queue_size=1)
		self.handle_pub = rospy.Publisher('/handle/command', Float32, queue_size=1)
		self.drive_pub = rospy.Publisher('/drive_wheel/command', Float32, queue_size=1)

		rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
		rospy.Subscriber('/stand1/joint_state', JointState, self.stand1_callback)
		rospy.Subscriber('/stand2/joint_state', JointState, self.stand2_callback)
		rospy.Subscriber('/handle/joint_state', JointState, self.handle_callback)
		self.flywheel_vel=0
		rospy.Subscriber('/flywheel/joint_state', JointState, self.flywheel_callback)
		rospy.Subscriber('/clock', Clock, self.clock_callback)

		# wasd=cv2.imread("wads_img.jpg")
		img = np.zeros((100,100,3), dtype=np.uint8)
		cv2.imshow("use w-a-s-d", img)
		cv2.waitKey(1)

	def imu_callback(self, msg):
		self.sbb_orientation_quaternion[0] = msg.orientation.x
		self.sbb_orientation_quaternion[1] = msg.orientation.y
		self.sbb_orientation_quaternion[2] = msg.orientation.z
		self.sbb_orientation_quaternion[3] = msg.orientation.w
		self.angular_vel[0]=msg.angular_velocity.x
		self.angular_vel[1]=msg.angular_velocity.y
		self.angular_vel[2]=msg.angular_velocity.z

	def stand1_callback(self,msg):
		self.stand1_angle=msg.position[0]
		self.stand1_vel=msg.velocity[0]

	def stand2_callback(self,msg):
		self.stand2_angle=msg.position[0]
		self.stand2_vel=msg.velocity[0]

	def handle_callback(self,msg):
		self.handle_angle=msg.position[0]
		self.handle_vel=msg.velocity[0]
	
	def flywheel_callback(self,msg):
		self.flywheel_vel=msg.velocity[0]

	def clock_callback(self, msg):
		self.time=msg.clock.secs
	
	def func(self,t,y):	# y is theta
		return y

	def pid(self):
		(self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
		print("angle: {:.5f}, stand1 angle: {:.5f}, stand2 angle: {:.5f}, handle angle: {:.5f}".format(self.sbb_orientation_euler[1], self.stand1_angle, self.stand2_angle, self.handle_angle))
		# print("current angle: ", np.rad2deg(self.sbb_orientation_euler[1]))
		# print("stand2 angle: ",np.rad2deg(self.stand2_angle))
		# print("handle angle: ",np.rad2deg(self.handle_angle))
		self.curr_angle = self.sbb_orientation_euler[1]

		# for plot
		#-----
		self.angles.append(self.sbb_orientation_euler[1])
		self.stand1s.append(self.stand1_angle)
		self.stand2s.append(self.stand2_angle)
		self.handles.append(self.handle_angle)

		plt.plot(self.angles)
		plt.pause(0.001)
		# plt.show()
		#-----

		# self.speed, self.angle = getdata(self.speed, self.angle)
		x=cv2.waitKey(1) & 0xFF
		# print(x)
		if x == ord('w'):
			self.speed+=1
		elif x == ord('s'):
			self.speed-=1
			# rospy.loginfo("s")
		elif x == ord('a'):
			self.angle-=np.pi/180
			# rospy.loginfo("a")
		elif x == ord('d'):
			self.angle+=np.pi/180
			# rospy.loginfo("d")
		elif x == ord(' '):
			self.speed=0
			self.angle=0
			print("space")
		# self.speed, self.angle = cv2.waitKey(1), cv2.waitKey(1)
		# print(self.speed,self.angle)

		self.drive_cmd.data=self.speed

		# if (0.06<abs(self.curr_angle)<0.09):
		# 	stand1_desired_angle=0.2
		# 	stand2_desired_angle=-0.2
		# else:
		# 	stand1_desired_angle=0.09
		# 	stand2_desired_angle=-0.09
		stand1_desired_angle=0.2
		stand2_desired_angle=-0.2
		if(abs(self.curr_angle)<0.02):
			stand1_desired_angle=0.2
			stand2_desired_angle=-0.2


		# stand1_desired_angle=0.09
		# stand1_desired_angle=0.2
		self.stand1_cmd.data=-2*(self.stand1_angle-stand1_desired_angle)
		# self.stand1_cmd.data=-100

		# stand2_desired_angle=-0.09
		# stand2_desired_angle=-0.2
		self.stand2_cmd.data=-2*(self.stand2_angle-stand2_desired_angle)
		# self.stand2_cmd.data=100

		handle_desired_angle=self.angle
		self.handle_cmd.data=-5*(self.handle_angle-handle_desired_angle)



		# if(self.first):
		# 	state=[self.curr_angle]
		# 	self.ode.set_initial_value(state,self.time)
		# 	self.first=False
		# else:
		# 	newstate=self.ode.integrate(self.time)
		# 	self.integral=newstate[0]

		self.t1=self.time
		self.integral+=self.curr_angle*(self.t1-self.t0)
		self.t0=self.time
		if (abs(self.flywheel_vel)<90):
			self.flywheel_cmd.data-= 0*self.integral + 8*self.curr_angle + 1*self.angular_vel[0]
		else:
			print("flywheel velocity reached it's max!! reducting it slowly...")
			self.decrease_vel()

		# if (abs(self.flywheel_vel)<90):
		# 	if(0.07<abs(self.curr_angle)<0.1):
		# 		self.flywheel_cmd.data-=15*self.curr_angle + 2*self.angular_vel[0]
		# 	else:
		# 		self.flywheel_cmd.data-=10*self.curr_angle + 2*self.angular_vel[0]
		# else:
		# 	print("flywheel velocity reached it's max!! reducting it slowly...")
		# 	self.decrease_vel()

		self.stand1_pub.publish(self.stand1_cmd)
		self.stand2_pub.publish(self.stand2_cmd)
		self.handle_pub.publish(self.handle_cmd)
		self.flywheel_pub.publish(self.flywheel_cmd)
		self.drive_pub.publish(self.drive_cmd)

	def decrease_vel(self):
		while(abs(self.flywheel_vel)>2):
			self.drive_cmd.data=0
			self.handle_cmd.data=0
			if(self.flywheel_vel>0):
				self.flywheel_cmd.data=self.flywheel_vel-1
			if(self.flywheel_vel<0):
				self.flywheel_cmd.data=self.flywheel_vel+1
			
			self.drive_pub.publish(self.drive_cmd)
			self.handle_pub.publish(self.handle_cmd)
			self.flywheel_pub.publish(self.flywheel_cmd)
			print(self.flywheel_vel)
			rospy.sleep(0.1)
		

if __name__ == '__main__':

	sbb = sbb()
	r = rospy.Rate(sbb.sample_rate)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		
		try:    
			sbb.pid()
			r.sleep()
		except rospy.exceptions.ROSTimeMovedBackwardsException:
			pass