#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose, Pose2DStamped

import os
import rospkg
import rospy
import yaml
import time
from twisted.words.protocols.oscar import CAP_SERV_REL
from math import sqrt, sin, cos

class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospack = rospkg.RosPack()
		self.car_cmd_pub = rospy.Publisher("~car_cmd",
				Twist2DStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size=1)
		self.subscriber = rospy.Subscriber("~detection",
				BoolStamped, self.callback,  queue_size=1)
		self.sub_vehicle_pose = rospy.Subscriber("~vehicle_pose", VehiclePose, self.cbPose, queue_size=1)
		self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)
		
		self.v_gain = 1
		self.vehicle_pose_msg_temp = VehiclePose()
		#self.vehicle_pose_msg_temp = Pose2DStamped()
		self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
		#self.time_temp = rospy.Time.now()
		self.v_rel = 0
		self.v = 0
		self.detection = False
		self.v_error_temp = 0
		self.I = 0
		self.v_follower = 0
		self.rho_temp = 0

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def callback(self, data):
#                 vehicle_too_close = BoolStamped()
#                 vehicle_too_close.header.stamp = data.header.stamp
# 		if not data.data:
# 			vehicle_too_close.data = False
# 		else:
# 			vehicle_too_close.data = True
# 		self.publishCmd(data.header.stamp)
# 		self.vehicle_detected_pub.publish(vehicle_too_close)
		vehicle_detected_msg_out = BoolStamped()
		vehicle_detected_msg_out.header.stamp = data.header.stamp
		vehicle_detected_msg_out.data = data.data
		self.vehicle_detected_pub.publish(vehicle_detected_msg_out)
		self.detection = data.data
		
		if  not data.data:
			self.v_gain = 1
			self.P = 0
			self.I = 0
			
		
	def cbPose(self, vehicle_pose_msg):
		d_desired = 0.2
		#distance_error_tolerance = 0.04
		d_min = 0.3
		Kp = 0.5
		Ki = 0.0
		Kd = 0.00
		
		time = rospy.Time.now()
		
		#rho = sqrt(vehicle_pose_msg.x**2 + vehicle_pose_msg.y**2)
		
		Ts = (vehicle_pose_msg.header.stamp - self.vehicle_pose_msg_temp.header.stamp).to_sec()
		#Ts2 = (time - self.time_temp).to_sec()
# 		print("-----")
# 		print(Ts)
# 		print(Ts2)
		self.vehicle_pose_msg_temp.header.stamp = vehicle_pose_msg.header.stamp
		#print(Ts)
		if Ts > 3:
			self.v_rel = 0
			self.v = 0
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			#self.rho_temp = rho
			self.v_error_temp = 0
			self.I = 0
		else:
			self.v_rel = (vehicle_pose_msg.rho.data - self.vehicle_pose_msg_temp.rho.data)/Ts
			v_leader = self.v_follower + self.v_rel
			#if (vehicle_pose_msg.rho.data - d_desired) > distance_error_tolerance:
			delta_v = (vehicle_pose_msg.rho.data - d_desired)/Ts
			#else:
			#	delta_v = 0
			v_des = v_leader + delta_v
			
# 			print("v leader")
# 			print(v_leader)
# 			print("v follower")
# 			print(self.v_follower)
# 			print("delta v")
# 			print(delta_v)
# 			print("v_rel")
# 			print(self.v_rel)
# 			print("rho")
# 			print(vehicle_pose_msg.rho)
# 			print("psi")
# 			print(vehicle_pose_msg.psi)
			
			v_error = v_des - self.v_follower

			self.P = Kp*v_error
			self.I = self.I + Ki * (v_error + self.v_error_temp)/2.0*Ts 
			self.D = Kd * (v_error + self.v_error_temp)/Ts 
			self.v = self.P + self.I + self.D
			
			if self.v < 0:
				self.v = 0
			
			#self.rho_temp = rho
			self.v_error_temp = v_error
			self.v_temp = self.v
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			#print(self.v)
			
			#self.time_temp = time
				
# 		v_gain_max = 1.5
# 		if d_min > vehicle_pose_msg.rho.data:
# 			self.v_gain = 0
# 		else:
# 			self.v_gain = (vehicle_pose_msg.rho.data - d_min)/(d_desired - d_min)
# 			if self.v_gain > v_gain_max:
# 				self.v_gain = v_gain_max
		
	def cbCarCmd(self, car_cmd_msg):
		car_cmd_msg_current = Twist2DStamped()
		car_cmd_msg_current = car_cmd_msg
		#car_cmd_msg_current.v = self.v_gain * car_cmd_msg_current.v
		if self.detection:
			car_cmd_msg_current.v = self.v
			print(self.v)
		self.v_follower = car_cmd_msg_current.v	
		#car_cmd_msg_current.omega = 0.0
		self.car_cmd_pub.publish(car_cmd_msg_current)
		#print(self.v_gain)
		

	def publishCmd(self,stamp): 
		cmd_msg = Twist2DStamped()
                cmd_msg.header.stamp = stamp
		cmd_msg.v = 0.0
		cmd_msg.omega = 0.0
		self.car_cmd_pub.publish(cmd_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

