import rclpy #importing ros
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from angle_helpers import euler_from_quaternion
from math import pi, sin, cos, atan2


class obstacle_avoider(Node):
    """
    A class used to dodge projectiles. The neato moves forward until it reaches
    an object too close to it at which point it stops and evaluates its next options.
    The first option is to check if it can turn 90 degrees either left or right with
    it choosing whichever one based on which one has more open space. If both left
    and right have objects too close, it will turn 180 degrees and move in the other
    direction.

    ...

    Methods
    -------
    enter_state()
        Keeps track of when the neato changes states as well as redefining what state
        the neato is in.
    run_loop()
        Runs the main code of the neato where it constantly evaluates which state it
        should be based on the outside parameters.
    rad_difference()
        Determines the amount of degrees the neato has moved from it starts turning based
        on the global coordinate system. This allows us to tell the neato to turn a
        specific amount in run_loop.
    determine_target()
        Because the neato's odometry coordinate system is -0 to 0 instead of 360 to 0,
        we need this function to tell when the neato is changing from positive or negative
        frames, which allows rad_difference() to properly calculate the angle.
    turn_left()
        Sends a ros message for the neato to turn left.
    turn_right()
        Sends a ros message for the neato to turn right.
    read_scan()
        Pulls the forward, left, and right distances from the lidar subscriber.
    neato_stop()
        Tells the neato to stop moving.
    check_odom()
        Pulls the odometry of the neato within the global coordinate frame.
    """
    def __init__(self):
        """
        Initializes the laser scan and odometry subscribers and movement
        publishers. Also defines all of the initial constants used throughout
        the functions.

        Args:

        Returns:
        """
        super().__init__("avoider")
        # create a subscriber to receive laser scan data
        self.create_subscription(LaserScan, 'scan', self.read_scan, 10)
        # create a subscriber to listen to the current position of the neato
        self.create_subscription(Odometry,"odom",self.check_odom,10 )

        # create a publisher for all movement
        self.movement = self.create_publisher(Twist, 'cmd_vel', 10)
        # run the run_loop function
        self.create_timer(0.1, self.run_loop)
        # create a placeholder for the range value of the x_axis
        self.x_axis = None
        # create a placeholder for the range value of the y_axis
        self.y_axis = None
        # create a placeholder for the range value of the -y_axis
        self.neg_y_axis = None
        # the current orientation (updated every loop)
        self.current_yaw = None
        # the starting orientation (used by some of the states)
        self.starting_yaw = None
        # the target orientation (will either be plus 90 or 180)
        self.target_yaw = None

        # set parameters for how fast the neato will turn and move rad/s
        self.angular_velocity = 0.25
        # set the linear velocity
        self.forward_velocity = .25
        # set the distance in meters, closest neato can approach obstacle
        self.dodge_distance = .65
        # state variable to control behavior
        # valid states are 'forward', 'stop', 'right', 'left', '180'
        self.state = 'stop'

    def enter_state(self, new_state):
        """
        Keeps track of what state the neato is changing to as well as defining
        any initial constants required to keep track of whether the neato has
        fulfilled a state.

        Args:
            new_state: a string representing what state the neato should change to
        Returns:
            new_state: a string specifying what state the neato is now in
        """

        if new_state == '180':
            self.starting_yaw = self.current_yaw
            self.determine_target('180', pi)
        elif new_state == 'left':
            self.starting_yaw = self.current_yaw
            self.determine_target('left', pi/2)
        elif new_state == 'right':
            self.starting_yaw = self.current_yaw
            self.determine_target('right', pi/2)
        return new_state

    def run_loop(self):
        """
        Runs all of the logic checks to determine which state the neato
        needs to be in at any time. Also executes all behavior relevant
        to the current state.

        Args:

        Returns:

        """

        # make sure we are receiving values, otherwise restart
        if self.x_axis is None or self.y_axis is None or self.neg_y_axis \
            is None or self.current_yaw is None:
            return
        # check if we should transition states
        if self.state == 'stop':
            # if the robot has space go forward
            if self.x_axis > self.dodge_distance:
                self.state = self.enter_state('forward')
            # if there not enough space check if it can turn
            elif self.y_axis < self.dodge_distance and self.neg_y_axis < self.dodge_distance:
                self.state = self.enter_state("180")
            # if there is enough space, check to turn left
            elif self.y_axis > self.neg_y_axis:
                self.state = self.enter_state("left")
            # if there is not enough space left, turn right
            else:
                self.state = self.enter_state("right")
        elif self.state == 'forward':
            if self.x_axis < self.dodge_distance:
                self.state = self.enter_state('stop')
        elif self.state == '180':
            # turn for 180 degrees
            if self.rad_difference() >= pi:
                self.state = self.enter_state("stop")
        elif self.state == 'left':
            # turn left 90 degrees
            if self.rad_difference() >= pi/2:
                self.state = self.enter_state("stop")
        elif self.state == 'right':
            # turn right 90 degrees
            if self.rad_difference() >= pi/2:
                self.state = self.enter_state("stop")

        # execute behavior
        if self.state == 'stop':
            self.neato_stop()
        elif self.state == 'forward':
            # create an empty ros movement message
            msg = Twist()
            # tell the neato to move forward
            msg.linear.x = self.forward_velocity
            # send the message through the publisher
            self.movement.publish(msg)
        elif self.state == '180':
            # turn left
            self.turn_left()
        elif self.state == 'left':
            # turn left
            self.turn_left()
        elif self.state == 'right':
            # turn right
            self.turn_right()

    def rad_difference(self):
        """
        Based on the desired target radian and current radian,
        determines the relative radial difference.

        Args:

        Returns:
            difference: a float signifying the radial difference from
            when a neato started turning and its current orientation.

        """

        if self.starting_yaw > 0 and self.target_yaw > 0:
            # if both are in the positive quadrant, make sure their value is positive
            difference = atan2(sin(self.current_yaw - self.starting_yaw),
                               cos(self.current_yaw - self.starting_yaw))
            return difference
        elif self.starting_yaw < 0 and self.target_yaw < 0:
            # if both are in the negative quadrant, make sure the difference is positive
            difference = abs(atan2(sin(self.current_yaw - self.starting_yaw),
                               cos(self.current_yaw - self.starting_yaw)))
            return difference
        elif self.starting_yaw < 0 and self.target_yaw > 0:
            if self.state == "left" or self.state == "180":
                # make sure the angle difference is positive
                difference = atan2(sin(self.current_yaw + pi/2 - self.starting_yaw + pi/2),
                                   cos(self.current_yaw + pi/2 - self.starting_yaw + pi/2))
                return difference
            else:
                # make sure the angle difference is positive if it is turning right
                difference = atan2(sin(self.current_yaw - self.starting_yaw + 2*pi),
                                   cos(self.current_yaw - self.starting_yaw + 2*pi))
                return difference
        elif self.starting_yaw > 0 and self.target_yaw < 0:
            if self.state == "left" or self.state == "180":
                # make sure the angle difference is positive
                difference = atan2(sin(self.current_yaw + 2*pi  - self.starting_yaw),
                                   cos(self.current_yaw + 2*pi - self.starting_yaw))
                return difference
            else:
                # make sure the angle difference is positive if it is turning right
                difference = atan2(sin(self.current_yaw + pi/2 - self.starting_yaw + pi/2),
                                   cos(self.current_yaw + pi/2 - self.starting_yaw + pi/2))
                return difference


    def determine_target(self, turn, amount):
        """
        Based on the desired target radian and current radian,
        determines the relative radial difference. Changes
        the target_yaw to -1 or 1 based on the frame of the target
        radian in comparison to the start point.

        Args:
            turn: string representing what turn state the neato is starting.
            amount: float representing the radian amount the neato will turn.

        Returns:

        """

        # determines if the target will be negative or positive
        # amount can either be pi/2 or pi, determined by turn
        # determine the target based on whether
        # the neato is yawing left or right

        if turn == "right":
            # turning right is the same as subtracting an angle
            self.target_yaw = self.current_yaw - amount
            # account for if the target is in the negative quadrant
            if self.target_yaw > -pi:
                self.target_yaw = -1
            else:
                self.target_yaw = 1
        elif turn == "left" or turn == "180":
            # turning left is the same as adding an angle
            self.target_yaw = self.current_yaw + amount

            # account for if target is in the positive quadrant
            if self.target_yaw < pi and self.target_yaw > 0:
                self.target_yaw = 1
            # otherwise it is in the negative quadrant
            else:
                self.target_yaw = -1

    def turn_left(self):
        """
        Tells the neato to turn counter clockwise. Velocity
        is determined in init.

        Args:

        Returns:

        """
        # create a blank movement ros message
        msg = Twist()
        # populate it with the defined angular velocity
        msg.angular.z = self.angular_velocity

        self.movement.publish(msg)

    def turn_right(self):
        """
        Tells the neato to turn clockwise. Velocity
        is determined in init.

        Args:

        Returns:

        """
        # create a blank movement ros message
        msg = Twist()
        # populate it with the defined angular velocity
        msg.angular.z = -self.angular_velocity

        self.movement.publish(msg)

    def read_scan(self, msg):
        """
        Finds the distance of any objects in front of,
        to the left, and to the right of the neato.

        Args:
            msg: ros object that contains all of the incoming
            sensor data.

        Returns:

        """
        # find the x axis range
        self.x_axis = msg.ranges[0]
        # find the y axis range
        self.y_axis = msg.ranges[90]
        # find the -y axis range
        self.neg_y_axis = msg.ranges[270]

    def neato_stop(self):
        """
        Tells the neato to stop moving.

        Args:
            msg: ros object that contains all of the incoming
            sensor data.

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()
        # send the message through the publisher
        self.movement.publish(msg)

    def check_odom (self,msg):
        """
        Finds the orientation of the neato with respect to the
        global frame.

        Args:
            msg: ros object that contains all of the incoming
            sensor data.

        Returns:

        """

        # harvest the odometry data of the neato from the subscriber
        self.current_position = msg.pose.pose.position
        # calculate the neato's angle
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation.x,
                                                 msg.pose.pose.orientation.y,
                                                 msg.pose.pose.orientation.z,
                                                 msg.pose.pose.orientation.w)[2]

def main(args=None):
    rclpy.init(args=args)
    node = obstacle_avoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()