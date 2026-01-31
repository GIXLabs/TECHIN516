import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# TODO: what do you need to import to get the right message type?


class PIDController():
    def __init__(
        self, 
        kp: float, 
        ki: float, 
        kd: float, 
        setpoint: float,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._prev_error = 0.0
        self._integral = 0.0
    
    def __call__(
        self, 
        system_measurement: float,
        dt: float
    ) -> float:
        """ compute PID output """
        assert dt > 0, "time must be positive"
        error = self.setpoint - system_measurement
        
        # proportional
        p_out = # TODO: proportional control is the kp term multiplied by the error

        # integral
        # integral control is based on the increasing area between the goal and error states
        # to help with steady-state errors when other controls bring the state parallel to the goal
        self._integral += # TODO: add the "area" of error over time to the class' _integral total
        i_out = # TODO: multiply the ki term by the total integral

        # derivative
        # derivative control is based on the slope of the system's state
        derivative = np.gradient() # TODO: use np.gradient to calculate the derivative between 
        # the previous error and the current error over time
        d_out = # TODO: multiply the kd term by the derivative at the current error

        # output
        self.prev_error = error
        out = p_out + i_out + d_out
        return out


class WallFollowPIDNode(Node):
    def __init__(self):
        super().__init__('wall_follow_pid_node')

        self.cmd_pub = self.create_publisher(<msg>, '/cmd_vel', 10) # TODO: what type of message?
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_scan = None
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=0.5)
        self.prev_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        if self.last_scan is None:
            return
        scan_ranges = np.array(self.last_scan.ranges)
        scan_ranges[np.isinf(scan_ranges)] = self.last_scan.range_max
        n_ranges = len(scan_ranges) # this might help you get the indicies
        angle_increment = self.last_scan.angle_increment

        front_index = # TODO: what is the index of the range value towards the front of the robot?
        # add +/- 10 degrees / 0.17 radians to look a little to each side as well
        front_indices = np.arange(int(front_index - (0.17 / angle_increment)),
                                  int(front_index + (0.17 / angle_increment)))
        dist_front = np.min(scan_ranges[front_indices])
    
        right_index = # TODO: what is the index of the range value towards the right of the robot?
        dist_right = scan_ranges[right_index]

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        pub_msg = # TODO: what type of message should you publish to drive the robot?
        if dist_front < 1:
            # TODO: how should the twist message change if there's a wall approaching?
            # the robot should always keep driving - so how should it turn?
        else:
            z = self.pid(dist_right, dt)
            pub_msg.angular.z = z
            pub_msg.linear.x = 0.1
        self.cmd_pub.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
