import rospy
import numpy as np
import rospy_crazyflie.crazyflie_client as crazyflie_client
import threading
from math import sin,cos,sqrt,atan2,acos,pi
from geometry_msgs.msg import PoseStamped
from pynput import keyboard
from std_msgs.msg import Float64MultiArray

cf_num = 3
cf_odom,client,odom_sub = {},{},{}
flag = 0
tl = 0

def odom_cb(msg, i):
    global cf_odom
    cf_odom[i] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

def takeoff():
    client[2].take_off(.2)
    client[1].take_off(.2)
    client[0].take_off(.2)
    
    while not cf_num-1 in cf_odom:
        pass

    while len(cf_odom[2])<2:
        if len(cf_odom[2])>1:
            break
        pass

    while rospy.get_param(crazyflies[0] + "_is_flying") == 0 or rospy.get_param(crazyflies[1] + "_is_flying") == 0 or rospy.get_param(crazyflies[2] + "_is_flying") == 0:
        pass
    print("takeoff")

def hover():
	client[2]._set_vel_setpoint(0.5*(des_x-cf_odom[2][0]),0.5*(1-cf_odom[2][1]),0.5*(des_z-cf_odom[2][2]),0)
	client[1]._set_vel_setpoint(0.5*(des_x-cf_odom[1][0]),0.5*(0-cf_odom[1][1]),0.5*(des_z-cf_odom[1][2]),0)
	client[0]._set_vel_setpoint(0.5*(des_x-cf_odom[0][0]),0.5*(-1-cf_odom[0][1]),0.5*(des_z-cf_odom[0][2]),0)

def on_press(key):
    global flag
    flag = key.char

if __name__ == "__main__":
    try:
        rospy.init_node('moo_controller')
        
        # Connect to the crazyflie
        crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
        for i in range(cf_num):
            client[i] = crazyflie_client.CrazyflieClient(crazyflies[i])
            odom_sub[i] = rospy.Subscriber("/vrpn_client_node/"+ crazyflies[i] +"/pose", PoseStamped, odom_cb, i, queue_size=10)

        # Causes the crazyflie to takeoff to height of .2 meters
        takeoff()

        print("start to hover")
        rate = rospy.Rate(3)
        des_x = -2
        des_z = 0.5
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        while not rospy.is_shutdown():
            if flag == ("a"):
                des_x = -1
                des_z = 1
            elif flag == ("b"):
                des_x = 0
                des_z = 0.5
            elif flag == ("c"):
                des_x = 1
                des_z = 1.5
            elif flag == ("d"):
                des_x = -1.5
                des_z = 0.5
            elif flag == ("e"):
                rate = rospy.Rate(50)
                des = -0.5
            elif flag == ("f"):
                rate = rospy.Rate(25)
                des = -0.25
            elif flag == ("g"):
                rate = rospy.Rate(20)
                des = 0
            elif flag == ("h"):
                rate = rospy.Rate(15)
                des = -0.25
            elif flag == ("i"):
                rate = rospy.Rate(10)
                des = -0.5
            elif flag == ("j"):
                rate = rospy.Rate(5)
                des = -0.75
            elif flag == ("k"):
                rate = rospy.Rate(3)
                des = -1

            hover()
            print(1/(rospy.Time.now().to_sec() - tl))
            tl = rospy.Time.now().to_sec()
            rate.sleep()
        
    except rospy.ROSInterruptException:        
        # Causes the crazyflie to land
        client[2].land()
        client[1].land()
        client[0].land()

        # Waits until current command is complete
        client[2].wait()
        client[1].wait()
        client[0].wait()

        # Exit program
        del client[2]
        del client[1]
        del client[0]
        
