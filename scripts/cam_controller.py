import rospy
import numpy as np
import rospy_crazyflie.crazyflie_client as crazyflie_client
import gurobipy as gp
from math import sin,cos,sqrt,atan2,acos,pi
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from gurobipy import GRB
from pyquaternion import Quaternion

P1,P2,P3,Pc,Pr,Pb,thetac,A,b = None,None,None,None,None,None,None,None,None
fx,fy,lx,ly = 565.6,565.6,640,480
x_fov_wealth = 3*pi/180
y_fov_wealth = 3*pi/180
height_l = 0.2
height_u = 0.5
d_safe_car = 0.7
d_measuring = 2.2 # optimal
#d_measuring = 1.5 # worst
d_safe_uav = 0.7
d_communication = 20
cf_odom = None
m,x = None,None
gamma = 1
gain = 1

def odom_cb(msg):
    global cf_odom
    q_ = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
    if q_.yaw_pitch_roll[0] < -1.57:
        tc = -3.14 - q_.yaw_pitch_roll[0]
    else:
        tc = q_.yaw_pitch_roll[0]
    cf_odom = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, tc]

def odom(msg):
	global P1,P2,P3,Pc,Pr,Pb,A,b,thetac
	
	Pc = np.array(msg.data[18:21])
	Pr = np.array(msg.data[21:24])
	Pb = np.array(msg.data[24:27])
	P1 = np.array(msg.data[0:3])
	P2 = np.array(msg.data[6:9])
	P3 = np.array(msg.data[12:15])
	thetac = msg.data[27]

	nc = np.array([cos(thetac),sin(thetac),0])
	nc_dot = np.array([-sin(thetac),cos(thetac),0])
	r1c = P1 - Pc
	r2c = P2 - Pc
	r3c = P3 - Pc

	A = np.array([ \
				  (-2*(Pc-P1)[:2]).tolist()+[0]*2, \
				  (-2*(Pc-P2)[:2]).tolist()+[0]*2, \
				  (-2*(Pc-P3)[:2]).tolist()+[0]*2, \
				  (2*(Pc-P1)[:2]).tolist()+[0]*2, \
				  (2*(Pc-P2)[:2]).tolist()+[0]*2, \
				  (2*(Pc-P3)[:2]).tolist()+[0]*2, \
				  (-2*(Pc-Pr)[:2]).tolist()+[0]*2, \
				  (-2*(Pc-Pb)[:2]).tolist()+[0]*2, \
				  (2*(Pc-Pr)[:2]).tolist()+[0]*2, \
				  (2*(Pc-Pb)[:2]).tolist()+[0]*2, \
				  np.concatenate((-(nc.dot(r1c)*r1c[:2]/np.linalg.norm(r1c[:2])**3-nc[:2]/np.linalg.norm(r1c[:2]))/sqrt(1 - nc.dot(r1c)**2/np.linalg.norm(r1c[:2])**2),[0],[-nc_dot.dot(r1c)/np.linalg.norm(r1c[:2])/sqrt(1 - nc.dot(r1c)**2/np.linalg.norm(r1c[:2])**2)])), \
				  np.concatenate((-(nc.dot(r2c)*r2c[:2]/np.linalg.norm(r2c[:2])**3-nc[:2]/np.linalg.norm(r2c[:2]))/sqrt(1 - nc.dot(r2c)**2/np.linalg.norm(r2c[:2])**2),[0],[-nc_dot.dot(r2c)/np.linalg.norm(r2c[:2])/sqrt(1 - nc.dot(r2c)**2/np.linalg.norm(r2c[:2])**2)])), \
				  np.concatenate((-(nc.dot(r3c)*r3c[:2]/np.linalg.norm(r3c[:2])**3-nc[:2]/np.linalg.norm(r3c[:2]))/sqrt(1 - nc.dot(r3c)**2/np.linalg.norm(r3c[:2])**2),[0],[-nc_dot.dot(r3c)/np.linalg.norm(r3c[:2])/sqrt(1 - nc.dot(r3c)**2/np.linalg.norm(r3c[:2])**2)])), \
				  np.concatenate((abs(r1c[2])*nc[:2]/nc.dot(r1c)**2/(1 + r1c[2]**2/nc.dot(r1c)**2),[-r1c[2]/nc.dot(r1c)/abs(r1c[2])/(1 + r1c[2]**2/nc.dot(r1c)**2)],[-abs(r1c[2])*nc_dot.dot(r1c)/nc.dot(r1c)**2/(1 + r1c[2]**2/nc.dot(r1c)**2)])), \
				  np.concatenate((abs(r2c[2])*nc[:2]/nc.dot(r2c)**2/(1 + r2c[2]**2/nc.dot(r2c)**2),[-r2c[2]/nc.dot(r2c)/abs(r2c[2])/(1 + r2c[2]**2/nc.dot(r2c)**2)],[-abs(r2c[2])*nc_dot.dot(r2c)/nc.dot(r2c)**2/(1 + r2c[2]**2/nc.dot(r2c)**2)])), \
				  np.concatenate((abs(r3c[2])*nc[:2]/nc.dot(r3c)**2/(1 + r3c[2]**2/nc.dot(r3c)**2),[-r3c[2]/nc.dot(r3c)/abs(r3c[2])/(1 + r3c[2]**2/nc.dot(r3c)**2)],[-abs(r3c[2])*nc_dot.dot(r3c)/nc.dot(r3c)**2/(1 + r3c[2]**2/nc.dot(r3c)**2)])), \
				  [0]*2+[-1]+[0], \
				  [0]*2+[1]+[0] \
				  ])

	b = np.array([ \
				  np.linalg.norm((Pc-P1)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pc-P2)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pc-P3)[:2])**2 - d_safe_car**2, \
				  d_measuring**2 - np.linalg.norm((Pc-P1)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pc-P2)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pc-P3)[:2])**2, \
				  np.linalg.norm((Pc-Pr)[:2])**2 - d_safe_uav**2, \
				  np.linalg.norm((Pc-Pb)[:2])**2 - d_safe_uav**2, \
				  d_communication**2 - np.linalg.norm((Pc-Pr)[:2])**2, \
				  d_communication**2 - np.linalg.norm((Pc-Pb)[:2])**2, \
				  atan2(lx,2*fx) - x_fov_wealth - acos(nc.dot(r1c)/np.linalg.norm(r1c[:2])), \
				  atan2(lx,2*fx) - x_fov_wealth - acos(nc.dot(r2c)/np.linalg.norm(r2c[:2])), \
				  atan2(lx,2*fx) - x_fov_wealth - acos(nc.dot(r3c)/np.linalg.norm(r3c[:2])), \
				  gain*(atan2(ly,2*fy) - y_fov_wealth - atan2(abs(r1c[2]),nc.dot(r1c))), \
				  gain*(atan2(ly,2*fy) - y_fov_wealth - atan2(abs(r2c[2]),nc.dot(r2c))), \
				  gain*(atan2(ly,2*fy) - y_fov_wealth - atan2(abs(r3c[2]),nc.dot(r3c))), \
				  gain*(Pc[2] - height_l), \
				  gain*(height_u - Pc[2]) \
				  ])*gamma

def qp_ini():
	global m,x
	
	m = gp.Model("qp")
	m.setParam("NonConvex", 2.0)
	m.setParam("LogToConsole",0)
	x = m.addVars(4,ub=0.3, lb=-0.3, name="x")

def addCons(i):
	global m

	m.addConstr(A[i,0]*x[0] + A[i,1]*x[1] + A[i,2]*x[2] + A[i,3]*x[3] <= b[i], "c"+str(i))

def takeoff():
    client_cam.take_off(.3)

    while rospy.get_param(crazyflies[2] + "_is_flying") == 0:
        pass
    print("takeoff")

def hover():

    while rospy.get_param("start_control") == 0:
        #client_cam._set_vel_setpoint(0.5*(-2.02-cf_odom[0]),0.5*(1.3-cf_odom[1]),0.5*(0.5-cf_odom[2]),-0.5*(-0.524-cf_odom[3])*180/pi) # worst
        client_cam._set_vel_setpoint(0.5*(-2.14-cf_odom[0]),0.5*(0.7-cf_odom[1]),0.5*(0.5-cf_odom[2]),-0.5*(-0.27-cf_odom[3])*180/pi) # optimal

def qpsolver():
	global x,camera_desired_pos
	
	#obj = -(x[0] - (P1 - Pc)[0])**2 - (x[1] - (P1 - Pc)[1])**2 + (x[2] - (P1 - Pc)[2])**2 - (x[0] - (P2 - Pc)[0])**2 - (x[1] - (P2 - Pc)[1])**2 + (x[2] - (P2 - Pc)[2])**2 - (x[0] - (P3 - Pc)[0])**2 - (x[1] - (P3 - Pc)[1])**2 + (x[2] - (P3 - Pc)[2])**2 + (thetac + x[3] - atan2((P1-Pc)[1],(P1-Pc)[0]))**2 + (thetac + x[3] - atan2((P2-Pc)[1],(P2-Pc)[0]))**2 + (thetac + x[3] - atan2((P3-Pc)[1],(P3-Pc)[0]))**2	# worst
	obj = (x[0] - (P1 - Pc)[0])**2 + (x[1] - (P1 - Pc)[1])**2 - (x[2] - (P1 - Pc)[2])**2 + (x[0] - (P2 - Pc)[0])**2 + (x[1] - (P2 - Pc)[1])**2 - (x[2] - (P2 - Pc)[2])**2 + (x[0] - (P3 - Pc)[0])**2 + (x[1] - (P3 - Pc)[1])**2 - (x[2] - (P3 - Pc)[2])**2 - (thetac + x[3] - atan2((P1-Pc)[1],(P1-Pc)[0]))**2 - (thetac + x[3] - atan2((P2-Pc)[1],(P2-Pc)[0]))**2 - (thetac + x[3] - atan2((P3-Pc)[1],(P3-Pc)[0]))**2	# optimal
	m.setObjective(obj)

	m.remove(m.getConstrs())
	
	for i in range (b.size):
		addCons(i)
		
	#print(Pc,thetac)
	m.optimize()
	
	if m.status == GRB.INFEASIBLE:
		m.feasRelaxS(1, False, False, True)
		m.optimize()

	optimal = m.getVars()
	#print(A.dot(np.array([optimal[0].X,optimal[1].X,optimal[2].X])) - b)

	client_cam._set_vel_setpoint(optimal[0].X,optimal[1].X,optimal[2].X,-optimal[3].X*180/pi)

if __name__ == "__main__":
    try:
        rospy.init_node('cam_controller')
        rospy.Subscriber('/state', Float64MultiArray, odom, queue_size=10)
        # Connect to the crazyflie
        crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')

        client_cam = crazyflie_client.CrazyflieClient(crazyflies[2])
        rospy.Subscriber("/vrpn_client_node/"+ crazyflies[2] +"/pose", PoseStamped, odom_cb, queue_size=10)

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
        while not rospy.is_shutdown():
            qpsolver()
            rate.sleep()
    except rospy.ROSInterruptException:
        # Causes the crazyflie to land
        client_cam.land()

        # Waits until current command is complete
        client_cam.wait()

        # Exit program
        del client_cam
