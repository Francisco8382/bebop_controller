#!/usr/bin/env python

## @file gazebo.py
## @brief Node file to get the drone position data from Gazebo.
##
## This node requires the following parameters to run.
## @param Topic Topic to publish the position data.
## @param GazeboRealTime Parameter used to compensate the gazebo real time which in most cases differs from the ROS time.

from subprocess import PIPE, Popen
from threading  import Thread
import sys
import numpy as np
import re
from queue import Queue, Empty
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

## Class of the publisher that gets the drone position data from the simulator.
class SphinxPublisher:
    def __init__(self):
        self.q = Queue()
        self.pos = np.zeros(3)
        self.att = np.zeros(3)
        self.cont = 1

        # Run the command
        ON_POSIX = 'posix' in sys.builtin_module_names
        command = "tlm-data-logger -r 0 inet:127.0.0.1:9060"
        p = Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
        
        # Create a thread which dies with main program
        t = Thread(target=self.process_output, args=(p.stdout, self.q))
        t.daemon = True 
        t.start()
        Topic = rospy.get_param('~Topic')
        GazeboRealTime = rospy.get_param('~GazeboRealTime')
        self.odom_pub = rospy.Publisher(Topic, PoseStamped, queue_size=10)
        Period = 1.0/(120.0*GazeboRealTime)
        rospy.Timer(rospy.Duration(Period), self.publish)
        rospy.spin() 

    ## Function that processes the console output and stores it in class variables.
    def process_output(self, out, queue):
        for line in iter(out.readline, b''):
            line = str(line)
            if ".worldPosition" in line:
                number = re.findall(r"[-+]?\d*\.\d+", line)[0]
                if ".x" in line:
                    self.pos[0] = float(number)
                if ".y" in line:
                    self.pos[1] = float(number)
                if ".z" in line:
                    self.pos[2] = float(number)


            if ".worldAttitude" in line:
                number = re.findall(r"[-+]?\d*\.\d+", line)[0]
                if ".x" in line:
                    self.att[0] = float(number)
                if ".y" in line:
                    self.att[1] = float(number)
                if ".z" in line:
                    self.att[2] = float(number)
                    
            queue.put(line)
        out.close()

    ## Function that publishes the drone position data in a PoseStamped message.
    def publish(self,event):
        try:  
            line = self.q.get_nowait()
        except Empty:
            self.q.queue.clear()
        qx, qy, qz, qw = quaternion_from_euler(self.att[0], self.att[1], self.att[2], 'sxyz')
        pose = PoseStamped()
        pose.header.seq = self.cont
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "bebop2"
        pose.pose.position.x = self.pos[1]
        pose.pose.position.y = self.pos[2]
        pose.pose.position.z = self.pos[0]
        pose.pose.orientation.x = qy
        pose.pose.orientation.y = qz
        pose.pose.orientation.z = qx
        pose.pose.orientation.w = qw
        self.cont += 1
        self.odom_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('SphinxPublisher')
    publisher = SphinxPublisher()