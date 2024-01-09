#!/usr/bin/env python

"""
Description: 
    1) read lidar_scan from topic
    2) find distance to the wall based on geometry described in UPenn lecture 6
    3) calculate the error for steering, based on difference of dist_wall
        and desired_dist_wall as input for PID
    4) Update steer_control (PID output)
    5) Publish velocity commands into /cmd_vel topic.
    

Authors:
    Tian Zhou (zhou338@purdue.edu)
    Maria E. Cabrera (cabrerm@purdue.edu)
    Teerachart Soratana (tsoratan@purdue.edu)

Date (updated): 
    Jan, 27, 2021

License: 
    GNU General Public License v3
"""
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from pid import PID
import numpy as np

class WALL_FOLLOWER:
    def __init__(self):
        # debug mode
        self.debug = False
        # init node
        rospy.init_node('wall_following', anonymous=True)
        # init PID for steer
        self.steer_pid = PID()
        # Subscribe to /scan topic to get laser values
        # use rospy.Subscriber('topic', msg_type, callback_function)
        self.<ADD SUBSCRIBER>
        # publisher for moving turtlebot
        # use rospy.Publisher('topic', msg_type, queue_size)
        self.cmd_vel_pub = <ADD PUBLISHER>
        # init twist
        self.twist = <ADD MSG_TYPE>
        # move forward constant velocity
        self.twist.linear.x = 0.2 
        # Set angle theta to look ahead for wall_following. 
        # It can take values from 20 to 70 degrees
        self.THETA_DEG = 30.0
        self.THETA_RAD = self.THETA_DEG * math.pi / 180.0

        # Initialize steer_e
        self.steer_e = 0.0


    def set_steer_PID(self, kp, ki, kd):
        # set steer PID control coefficients
        print "Set steer PID: "
        self.steer_pid.set_PID(kp = kp, ki = ki, kd = kd)

    def set_dist_right_wall(self, dist_right_wall):
        """
        set the desired distance to the right wall, in our case 0.5 m
        """
        if dist_right_wall < 0 or dist_right_wall > 1:
            print "Invalid dist_right_wall value %i, range [0, 1]" % dist_right_wall
            exit(-1)
        self.dist_right_wall = dist_right_wall

    def scan_callback(self, msg):
        """
        msg.ranges has 360 elements
        with each element being 1 degree
        msg.ranges[0] is straight ahead increasing counter-clock wise
        """

        # The following two value readings are important and need to be changed for 
        # different sensor placements and sensor value directions and ranges
        right = <SELECT msg.ranges element> # The right of the robot is at 270
        right_ahead_theta = msg.ranges[<ELEMENT NUM> + int(self.THETA_DEG)]
        
        dist_wall = self.calc_dist_to_wall(right, right_ahead_theta)
        steer_e = self.dist_right_wall - dist_wall

        if self.debug:
            print "right: %.3f, right ahead (theta): %.3f, dist_wall: %.3f" % \
                (right, right_ahead_theta, dist_wall)

        # Use the steer_e error to get the PID output control
        self.steer_pid.input_error(steer_e)
        steer_control = self.steer_pid.output_control() 
        if self.debug:
            print "steer_control: %.3f" % steer_control
        
        # Check the steer_control value is valid and assign it to the
        # angular portion of the twist message
        if not math.isnan(steer_control) or not math.isinf(steer_control):
            self.twist.angular.z = steer_control


        self.cmd_vel_pub.publish(self.twist)
        

    def calc_dist_to_wall(self, right, right_ahead_theta):
        """
        calculate the distance of the car to the right wall, according to geometry
        from the UPenn videos.
        """
        numerator = right_ahead_theta * math.cos(self.THETA_RAD) - right
        denominator = right_ahead_theta * math.sin(self.THETA_RAD)
        steer_rad = math.atan2(numerator,denominator) # alpha (steer_rad) in radian
        dist_to_wall = right * math.cos(steer_rad)
        if self.debug:
            print "steer_deg: %.3f, dis_to_wall: %.3f" % \
                (steer_rad * 180.0 / math.pi, dist_to_wall)

        return dist_to_wall

        def main():
            tbot3 = WALL_FOLLOWER()
            tbot3.debug = True
            tbot3.set_steer_PID(kp=1.1, ki=0.2, kd=2.5)
            tbot3.set_dist_right_wall(0.5)
            rospy.spin()


if __name__ == '__main__':
    main()