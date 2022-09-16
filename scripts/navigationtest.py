#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist

car1_cmd_vel,car2_cmd_vel,car3_cmd_vel = Twist(),Twist(),Twist()
time = 0

def start():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel,time
	car1_cmd_vel.linear.x = 0.05
	car2_cmd_vel.linear.x = 0.05
	car3_cmd_vel.linear.x = 0.05
	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)
	time = 1

def control():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel,time
	
	if time < 201:
		car1_cmd_vel.linear.x = 0.025
		car2_cmd_vel.linear.x = 0.025
		car3_cmd_vel.linear.x = 0.025
	elif time < 321:
		car1_cmd_vel.linear.x = 0.03
		car2_cmd_vel.linear.x = 0.03
		car3_cmd_vel.linear.x = 0.03
		car1_cmd_vel.angular.z = 0.09
		car2_cmd_vel.angular.z = 0.09
		car3_cmd_vel.angular.z = 0.09
	elif time < 601:
		car1_cmd_vel.linear.x = 0.025
		car2_cmd_vel.linear.x = 0.025
		car3_cmd_vel.linear.x = 0.025
	elif time < 721:
		car1_cmd_vel.linear.x = 0.035
		car2_cmd_vel.linear.x = 0.035
		car3_cmd_vel.linear.x = 0.035
		car1_cmd_vel.angular.z = -0.1
		car2_cmd_vel.angular.z = -0.1
		car3_cmd_vel.angular.z = -0.1
	elif time < 1201:
		car1_cmd_vel.linear.x = 0.04
		car2_cmd_vel.linear.x = 0.04
		car3_cmd_vel.linear.x = 0.04
	elif time < 1401:
		car1_cmd_vel.linear.x = 0.035
		car2_cmd_vel.linear.x = 0.025
		car3_cmd_vel.linear.x = 0.03
		car1_cmd_vel.angular.z = 0.05
		car2_cmd_vel.angular.z = 0.05
		car3_cmd_vel.angular.z = 0.05
	elif time < 1601:
		car1_cmd_vel.linear.x = 0.035
		car2_cmd_vel.linear.x = 0.025
		car3_cmd_vel.linear.x = 0.03
		car1_cmd_vel.angular.z = 0.05
		car2_cmd_vel.angular.z = 0.05
		car3_cmd_vel.angular.z = 0.05

	'''
	if time < 600:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
	'''
	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)
	time = time+1

def stop():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel
	car1_cmd_vel.linear.x = 0.0
	car2_cmd_vel.linear.x = 0.0
	car3_cmd_vel.linear.x = 0.0
	car1_cmd_vel.angular.z = 0.0
	car2_cmd_vel.angular.z = 0.0
	car3_cmd_vel.angular.z = 0.0
	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)

if __name__ == '__main__':
	try:
		rospy.init_node('navigation')
		car1_vel_pub = rospy.Publisher("/car1/cmd_vel",Twist,queue_size=1)
		car2_vel_pub = rospy.Publisher("/car2/cmd_vel",Twist,queue_size=1)
		car3_vel_pub = rospy.Publisher("/car3/cmd_vel",Twist,queue_size=1)
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			if time == 0:
				start()
			elif time < 1601:
				control()
			else:
				stop()
				rospy.set_param("stop_ukf",1)
				break
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
