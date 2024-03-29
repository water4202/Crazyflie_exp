import rospy_crazyflie.crazyflie_client as crazyflie_client
import gurobipy as gp
import rospy
import numpy as np
from math import sin,cos,sqrt,atan2,acos,pi
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from gurobipy import GRB

dt,t_l = 0,0
P1,P2,P3,Pc,Pr,Pb,A,b = None,None,None,None,None,None,None,None
sigma_u,sigma_v,sigma_ranging,sigma_bearing,sigma_alpha = 0.007,0.007,0.01,0.01,0.01
height_l = 0.2
height_u = 0.5
d_safe_car = 0.4
#d_measuring = 2.2 # optimal
d_measuring = 0.9 # worst
d_safe_uav = 0.7
d_communication = 20
cf_odom = None
m,x = None,None
#gamma = 0.4 # optimal
gamma = 0.5 # worst
gain = 1

def odom_cb(msg):
    global cf_odom
    cf_odom = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

def odom(msg):
	global P1,P2,P3,Pc,Pr,Pb,A,b
	
	Pc = np.array(msg.data[18:21])
	Pr = np.array(msg.data[21:24])
	Pb = np.array(msg.data[24:27])
	P1 = np.array(msg.data[0:3]) + dt*np.array(msg.data[3:6])
	P2 = np.array(msg.data[6:9]) + dt*np.array(msg.data[9:12])
	P3 = np.array(msg.data[12:15]) + dt*np.array(msg.data[15:18])

	A = np.array([ \
				  (-2*(Pb-P1)[:2]).tolist()+[0], \
				  (-2*(Pb-P2)[:2]).tolist()+[0], \
				  (-2*(Pb-P3)[:2]).tolist()+[0], \
				  (2*(Pb-P1)[:2]).tolist()+[0], \
				  (2*(Pb-P2)[:2]).tolist()+[0], \
				  (2*(Pb-P3)[:2]).tolist()+[0], \
				  (-2*(Pb-Pc)[:2]).tolist()+[0], \
				  (-2*(Pb-Pr)[:2]).tolist()+[0], \
				  (2*(Pb-Pc)[:2]).tolist()+[0], \
				  (2*(Pb-Pr)[:2]).tolist()+[0], \
				  [0]*2+[-1], \
				  [0]*2+[1], \
				  ])
	
	b = np.array([ \
				  np.linalg.norm((Pb-P1)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pb-P2)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pb-P3)[:2])**2 - d_safe_car**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P1)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P2)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P3)[:2])**2, \
				  np.linalg.norm((Pb-Pc)[:2])**2 - d_safe_uav**2, \
				  np.linalg.norm((Pb-Pr)[:2])**2 - d_safe_uav**2, \
				  d_communication**2 - np.linalg.norm((Pb-Pc)[:2])**2, \
				  d_communication**2 - np.linalg.norm((Pb-Pr)[:2])**2, \
				  gain*(Pb[2] - height_l), \
				  gain*(height_u - Pb[2]) \
				  ])*gamma

def takeoff():
    client_bearing.take_off(.3)

    while rospy.get_param(crazyflies[0] + "_is_flying") == 0:
        pass
    print("takeoff")

def hover():

    while rospy.get_param("start_control") == 0:
        client_bearing._set_vel_setpoint(0.5*(0.7-cf_odom[0]),0.5*(0.0-cf_odom[1]),0.5*(0.5-cf_odom[2]),0)
        #client_bearing._set_vel_setpoint(0.5*(0.1-cf_odom[0]),0.5*(-0.7-cf_odom[1]),0.5*(0.5-cf_odom[2]),0)

def qp_ini():
	global m,x
	
	m = gp.Model("qp")
	m.setParam("NonConvex", 2.0)
	m.setParam("LogToConsole",0)
	x = m.addVars(3,ub=0.3, lb=-0.3, name="x")

def addCons(i):
	global m

	m.addConstr(A[i,0]*x[0] + A[i,1]*x[1] + A[i,2]*x[2] <= b[i], "c"+str(i))

def qpsolver():
	global x

	obj = -(x[0] - (P1 - Pb)[0])**2 - (x[1] - (P1 - Pb)[1])**2 - (x[2] - (P1 - Pb)[2])**2 - (x[0] - (P2 - Pb)[0])**2 - (x[1] - (P2 - Pb)[1])**2 - (x[2] - (P2 - Pb)[2])**2 - (x[0] - (P3 - Pb)[0])**2 - (x[1] - (P3 - Pb)[1])**2 - (x[2] - (P3 - Pb)[2])**2 # worst
	#obj = (x[0] - (P1 - Pb)[0])**2 + (x[1] - (P1 - Pb)[1])**2 + (x[2] - (P1 - Pb)[2])**2 + (x[0] - (P2 - Pb)[0])**2 + (x[1] - (P2 - Pb)[1])**2 + (x[2] - (P2 - Pb)[2])**2 + (x[0] - (P3 - Pb)[0])**2 + (x[1] - (P3 - Pb)[1])**2 + (x[2] - (P3 - Pb)[2])**2 # optimal
	m.setObjective(obj)

	m.remove(m.getConstrs())
	
	for i in range (b.size):
		addCons(i)
	
	m.optimize()
	#print(m.computeIIS())
	optimal = m.getVars()
	#print(A.dot(np.array([optimal[0].X,optimal[1].X,optimal[2].X])) - b)
	#print(optimal[2].X)

	client_bearing._set_vel_setpoint(optimal[0].X,optimal[1].X,optimal[2].X,0)

if __name__ == "__main__":
    try:
        rospy.init_node('bearing_controller')
        rospy.Subscriber('/state', Float64MultiArray, odom, queue_size=10)
        # Connect to the crazyflie
        crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')

        client_bearing = crazyflie_client.CrazyflieClient(crazyflies[0])
        rospy.Subscriber("/vrpn_client_node/"+ crazyflies[0] +"/pose", PoseStamped, odom_cb, queue_size=10)

        rate = rospy.Rate(100)
        while cf_odom is None:
            rate.sleep()
        # Causes the crazyflie to takeoff to height of .2 meters
        takeoff()

        print("start to hover")
        hover()
        
        print("start controlling")

        while b is None:
            rate.sleep()

        qp_ini()
        t_l = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            dt = rospy.Time.now().to_sec() - t_l
            qpsolver()
            if rospy.get_param("stop_ukf") == 1:
                break
            t_l = rospy.Time.now().to_sec()
            rate.sleep()

        client_bearing.land()
        client_bearing.wait()
        del client_bearing
    except rospy.ROSInterruptException:
        # Causes the crazyflie to land
        client_bearing.land()

        # Waits until current command is complete
        client_bearing.wait()

        # Exit program
        del client_bearing
