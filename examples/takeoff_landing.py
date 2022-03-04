"""
Copyright (c) 2018, Joseph Sullivan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the <project name> project.
"""

"""
This is a demontration of how to make a crazyflie takeoff and land.
"""
import rospy
import time
import rospy_crazyflie.crazyflie_client as crazyflie_client
import sys
from geometry_msgs.msg import PoseStamped
import numpy as np

cf_num = 3

def odom_cb(msg, i):
    global cf_odom
    cf_odom[i] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

if __name__ == "__main__":
    rospy.init_node('rospy_crazyflie_example')
    cf_odom = {}
    client = {}
    odom_sub = {}
    # Connect to the crazyflie
    crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
    for i in range(cf_num):
        client[i] = crazyflie_client.CrazyflieClient(crazyflies[i])
        odom_sub[i] = rospy.Subscriber("/vrpn_client_node/"+ crazyflies[i] +"/pose", PoseStamped, odom_cb, i, queue_size=10)

    # Causes the crazyflie to takeoff to height of .5 meters
    
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
        print("wait takeoff")
    #while rospy.get_param(crazyflies[0] + "_is_flying") == 0 or rospy.get_param(crazyflies[1] + "_is_flying") == 0:

    #while not rospy.is_shutdown() and rospy.get_param(crazyflies[0] + "_is_flying") == 1 and rospy.get_param(crazyflies[1] + "_is_flying") == 1:
    while not rospy.is_shutdown() and rospy.get_param(crazyflies[0] + "_is_flying") == 1 and rospy.get_param(crazyflies[1] + "_is_flying") == 1 and rospy.get_param(crazyflies[2] + "_is_flying") == 1:
        client[2]._set_vel_setpoint(0.5*(-1.6-cf_odom[2][0]),0.5*(-0.5-cf_odom[2][1]),0.5*(0.5-cf_odom[2][2]),0)
        client[1]._set_vel_setpoint(0.5*(-1.3-cf_odom[1][0]),0.5*(0.0-cf_odom[1][1]),0.5*(0.5-cf_odom[1][2]),0)
        client[0]._set_vel_setpoint(0.5*(-1.0-cf_odom[0][0]),0.5*(0.5-cf_odom[0][1]),0.5*(0.5-cf_odom[0][2]),0)
        print("CF1 position error:",[-1.6-cf_odom[2][0],-0.5-cf_odom[2][1],0.5-cf_odom[2][2]])
        print("CF2 position error:",[-1.3-cf_odom[1][0],0.0-cf_odom[1][1],0.5-cf_odom[1][2]])
        print("CF3 position error:",[-1.0-cf_odom[0][0],0.5-cf_odom[0][1],0.5-cf_odom[0][2]])

        if np.linalg.norm(np.array([-1.6-cf_odom[2][0],-0.5-cf_odom[2][1],0.5-cf_odom[2][2]])) < 0.1 and np.linalg.norm(np.array([-1.3-cf_odom[1][0],0.0-cf_odom[1][1],0.5-cf_odom[1][2]])) < 0.1 and np.linalg.norm(np.array([-1.0-cf_odom[0][0],0.5-cf_odom[0][1],0.5-cf_odom[0][2]])) < 0.1:
            break

    

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

    sys.exit(0)
