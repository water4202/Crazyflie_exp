#! /usr/bin/python3
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
import sys
import time
import threading
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

from rospy_crazyflie.crazyflie_server.crazyflie_log import CrazyflieLog
from rospy_crazyflie.crazyflie_server.crazyflie_control import CrazyflieControl
import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Localization
from rospy_crazyflie.srv import GetCrazyflies, GetCrazyfliesResponse

fly_flag = 0

class CrazyflieServer:

    def __init__(self):
        # Ros stuff
        rospy.init_node("crazyflie_server")

        # Dictionaries to hold the crazyflies and related objects
        # self._address = rospy.get_param('~address')
        self._uris = rospy.get_param('~uris')
        print(self._uris)
        self._crazyflies = {}
        self._crazyflie_logs = {}
        self._controllers = {}
        self._odom = {}
        self.odom_sub = {}
        self.flag = 0

        cflib.crtp.init_drivers(enable_debug_driver=False)

        self._get_crazyflies_srv = rospy.Service (
            '~get_crazyflies',
            GetCrazyflies,
            self._get_crazyflies_cb
        )

        for name in self._uris.keys():
            uri = self._uris[name]
            parts = uri.split('/')
            channel = parts[3]
            cf = Crazyflie()
            cf.connected.add_callback(self._connected)
            cf.disconnected.add_callback(self._disconnected)
            cf.connection_failed.add_callback(self._connection_failed)
            cf.connection_lost.add_callback(self._connection_lost)
            cf.open_link(uri)
            self._crazyflies[uri] = (name, cf)

        for link_uri in self._crazyflies.keys():
            name = self._crazyflies[link_uri][0]
            self.odom_sub[name] = rospy.Subscriber("/vrpn_client_node/"+ name +"/pose", PoseStamped, self.odom_cb, link_uri, queue_size=10)
            self._odom[link_uri] = Localization(self._crazyflies[link_uri][1])
            self._crazyflies[link_uri][1].param.set_value('stabilizer.estimator', '2')
            self._crazyflies[link_uri][1].param.set_value('locSrv.extQuatStdDev', 0.06)
            self._crazyflies[link_uri][1].param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            self._crazyflies[link_uri][1].param.set_value('kalman.resetEstimation', '0')
            #self.odom_cb(0, link_uri)

    def _get_crazyflies_cb (self, request) :
        response = GetCrazyfliesResponse()
        for (name, crazyflie) in self._crazyflies.values():
            if crazyflie.is_connected():
                response.crazyflies.append(name)
        return response


    def _connected(self, link_uri):
        print('Connected to %s.' % (link_uri))
        if link_uri not in self._crazyflie_logs:
            log = CrazyflieLog(*self._crazyflies[link_uri])
            self._start_logs(log)
            self._crazyflie_logs[link_uri] = log
        if link_uri not in self._controllers:
            controller = CrazyflieControl(*self._crazyflies[link_uri])
            self._controllers[link_uri] = controller

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
        Crazyflie moves out of range)"""
        print('Connection to %s lost : %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        print('Reconnecting...')
        #self._crazyflies[link_uri][1].open_link(link_uri)

    def _disconnect_srv_cb(self, request):
        for link_uri in self._crazyflies.keys():
            (name, crazyflie) = self._crazyflies[link_uri]
            if name == request.name:
                self._crazyflie_logs[link_uri].stop_logs()
        return []

    def _start_logs(self, log):
        """ Starts all logs which are enabled in the loaded config file """
        pub_options = rospy.get_param('~pub_controller_rpy_rate')
        if pub_options['enable']:
            log.log_controller_rpy_rate(pub_options['period_in_ms'])

        pub_options = rospy.get_param('~pub_controller_rpyt')
        if pub_options['enable']:
            log.log_controller_rpyt(pub_options['period_in_ms'])

        pub_options = rospy.get_param('~pub_kalman_position')
        if pub_options['enable']:
            log.log_kalman_position_est(pub_options['period_in_ms'])

        pub_options = rospy.get_param('~pub_motor_power')
        if pub_options['enable']:
            log.log_motor_power(pub_options['period_in_ms'])

        pub_options = rospy.get_param('~pub_pos_ctl')
        if pub_options['enable']:
            log.log_pos_ctl(pub_options['period_in_ms'])

        pub_options = rospy.get_param('~pub_stabilizer')
        if pub_options['enable']:
            log.log_stabilizer(pub_options['period_in_ms'])

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(.1)
        print("shutting down")
        for link_uri in self._crazyflies.keys():
            cf = self._crazyflies[link_uri][1]
            print("closing link to %s" % link_uri)
            if cf.is_connected():
                try:
                    self._controllers[link_uri].land()
                except:
                    pass
                cf.close_link()
                del(cf)

    def odom_cb(self, msg, link_uri):
        self._odom[link_uri].send_extpos([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self._controllers != {}:
            if link_uri in self._controllers:
                if self._controllers[link_uri]._mc._thread != None:
                    rospy.set_param(self._crazyflies[link_uri][0] + "_is_flying",1)
        '''
        a = 0
        while not rospy.is_shutdown():
            if rospy.get_param("odom") == 1 and a == 0:
                time.sleep(4)
                self._odom[link_uri].send_extpos([0, 0, 0.5])
                a = 1
            if a == 1:
                self._odom[link_uri].send_extpos([0, 0, 0.7])
                a = 2
            if a == 2:
                self._odom[link_uri].send_extpos([0, 0, 0.3])
                a = 1
        '''
