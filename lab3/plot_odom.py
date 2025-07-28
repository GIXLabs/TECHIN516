#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import # TODO: what type of message should be subscribed to?
import matplotlib.pyplot as plt


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            <message>, # TODO: what type of message should be subscribed to?
            '</topic>', # TODO: what topic should be subscribed to?
            <function>, # TODO: which function should be used as the 'callback' function?
            <queue_size>) # TODO: what should the queue size be?
        self.subscription  # prevent unused variable warning
        # Lists to store the x and y data
        self.x_data = []
        self.y_data = []

    def odom_callback(self, msg):
        # Extract the x and y positions
        position = # TODO: retrieve the pose data from the message
        x = # TODO: retrieve the x data from the pose
        y = # TODO: retireve the y data from the pose
        # Log the position
        self.get_logger().info(f"Position -> x: {x}, y: {y}")
        # Append the x and y positions to the lists
        self.x_data.append(x)
        self.y_data.append(y)

    def plot_data(self):
        # Plot x and y data
        plt.figure()
        plt.plot(self.x_data, self.y_data, label='Odometry Path')
        plt.title('Odometry X-Y Path')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    odom_plotter = OdomPlotter()
    try:
        rclpy.spin(odom_plotter)
    except KeyboardInterrupt:
        # On shutdown, plot the data
        odom_plotter.plot_data()
    odom_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
