import rclpy #importing ros
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
import time

class obstacle_avoider(Node):
    def __init__(self):
        super().__init__("avoider")
        # create a subscriber to receive laser scan data
        self.create_subscription(LaserScan, 'scan', self.read_scan, 10)
        # create a publisher for all movement
        self.movement = self.create_publisher(Twist, "movement", 10)
        # run the run_loop function
        self.create_timer(0.1, self.run_loop)
        # create a placeholder for the range value of the x_axis
        self.x_axis = None
        # create a placeholder for the range value of the y_axis
        self.y_axis = None
        # create a placeholder for the range value of the -y_axis
        self.neg_y_axis = None

        # set parameters for how fast the neato will turn and move rad/s
        self.angular_velocity = 0.1
        # store the amount of time in seconds a 90 degree turn takes
        self.right_turn_time = (pi/2)/self.angular_velocity
        # set the linear velocity
        self.forward_velocity = .5
        # set the distance in meters, closest neato can approach obstacle
        self.dodge_distance = .5

    def run_loop(self):
        # scan lidar measurements
        self.read_scan()
        # create an empty ros movement message
        msg = Twist()
        # tell the neato to move forward
        msg.linear.x = self.forward_velocity
        # send the message through the publisher
        self.movement.publish(msg)

        # go forward until dodge distance is reached
        if self.x_axis < self.dodge_distance:
            self.neato_stop()
            # check whether to turn left or right
            

    def read_scan(self, msg):
        # find the x axis range
        self.x_axis = msg.ranges[0]
        # find the y axis range
        self.y_axis = msg.ranges[90]
        # find the -y axis range
        self.neg_y_axis = msg.ranges[270]


    def turn_left_90(self):
        # store the current time
        start_time = time.time()

        # create a movement message to send to ros
        msg = Twist()

        # assign the angular velocity
        msg.angular.z = self.angular_velocity

        # tell the neato to move 90 degrees counter clockwise
        while(time.time - start_time < self.right_turn_time):
            # use the publisher we set up to send the message to ros
            self.movement.publish(msg)

        # now tell the neato to stop
        self.neato_stop()

    def turn_right_90(self):
        # store the current time
        start_time = time.time()

        # create a movement message to send to ros
        msg = Twist()

        # assign the angular velocity, but negative
        msg.angular.z = -self.angular_velocity

        # tell the neato to move 90 degrees counter clockwise
        while(time.time - start_time < self.right_turn_time):
            # use the publisher we set up to send the message to ros
            self.movement.publish(msg)

        # now tell the neato to stop
        self.neato_stop()

    def neato_stop(self):
        # by default, all values for movement message are 0
        msg = Twist()
        # send the message through the publisher
        self.movement.publish(msg)
    