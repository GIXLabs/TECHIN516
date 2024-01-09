#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  8 23:42:25 2020

@author: zhitaoyu
"""
import numpy as np
#from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class odom_closeloop:
    def __init__(self):
        rospy.init_node("close_loop_odom", anonymous = True)
        # debug mode
        self.debug = False
        # TODO: Add the correct command to subscribe to the odom topic
        # Use rospy.Subscriber(topic,msg_type,callback_function)
        self.sub = <configure subscriber>
        # TODO: Add the correct command to publish velocity values
        # Use rospy.Publisher(topic,msg_type,queue_size typically 10)
        self.pub = <configure publisher>
        self.position_x_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.15
        # Set speed to stop
        self.motion_stop.linear.x = 0.0

    def odom_callback(self,msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)
        # This if guarantees that the list has at least 2 elements
        if len(self.position_x_list) > 2:
            # TODO: Fill out the if condition to check whether the last item on the list
            # is greater than the first one plus the desired distance to be traveled
            if (<CONDITION TO STOP>):
                self.pub.publish(self.motion_stop)
                rospy.signal_shutdown("Reached goal")
            else:
                self.pub.publish(self.motion_move)
                if self.debug == True:
                    print(msg.pose.pose)


if __name__ == '__main__':
   
    odom_cl = odom_closeloop()
    odom_cl.debug = True
    rospy.spin()
