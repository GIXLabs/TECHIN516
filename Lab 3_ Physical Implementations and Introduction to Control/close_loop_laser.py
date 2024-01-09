#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  8 23:42:25 2020

@author: zhitaoyu
"""
import numpy as np
#from matplotlib import pyplot as plt
import rospy
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class laser_closeloop:
    def __init__(self):
        rospy.init_node("laser_closeloop", anonymous = True)
        self.debug = False
        # TODO: Add the correct command to subscribe to the scan topic
        # Use rospy.Subscriber(topic,msg_type,callback_function)
        self.sub = <configure subscriber>
        # TODO: Add the correct command to publish velocity values
        # Use rospy.Publisher(topic,msg_type,queue_size typically 10)
        self.pub = <configure publisher>
        self.front_value_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.15
        # Set speed to stop
        self.motion_stop.linear.x = 0.0

    def scan_callback(self,msg):
        # The index of the front value might need to be changed.
        current_front_value = msg.ranges[0]
        #print 'current_front_value', current_front_value
        self.front_value_list.append(current_front_value)
        
        if len(self.front_value_list) > 2:
            # TODO: Fill out the if condition to check whether the last item on the list
            # is smaller than the substraction between the first one and the desired distance to be traveled
            if (<CONDITION TO STOP>):
                self.pub_move.publish(self.motion_stop)
                rospy.signal_shutdown("Reached goal")
            else:
                self.pub_move.publish(self.motion_move)
                if self.debug == True:
                    print("Value ahead: ", current_front_value)
                    print("Distance traveled: ", self.front_value_list[0] - self.front_value_list[-1])

if __name__ == '__main__':
    scan_cl = laser_closeloop()
    scan_cl.debug = True
    rospy.spin()
