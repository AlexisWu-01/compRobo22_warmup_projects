from turtle import position
import rclpy #importing ros
from rclpy.node import Node
from std_msgs.msg import String #datatype needed for ros to understand
from geometry_msgs.msg import Twist, Vector3 #for the neato
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from .angle_helpers import euler_from_quaternion
import math


class person_follower(Node):
    """
    A ros node to make the Neato find a person/ obstacle and follows its path.

    ...

    Methods
    -------
    Finds the center of obstacle coordinates (like center of mass)
    then turns and go towards the target position.

    """
    def __init__(self):
        super().__init__("publisher_node") #call parent class

        timer_period = 0.05 # the time at which send_msg() is called

        #Subscribe to laser scan: gets a list of distance from 360 degree laser scan
        self.create_subscription(LaserScan,'scan',self.process_scan,10)

        # Subscribe to bumper sensor: the neato would not go any further if it hits an obstacle
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        # Subscribe to position information from odometry
        self.create_subscription(Odometry,"odom",self.check_odom,10 )

        # Publish msg to neato, which is speed in this publisher
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)

        # Runs send_msg at the defined timer_period
        self.timer = self.create_timer(timer_period,self.send_msg)

        # Constant coefficients for proportional control
        self.kp = 0.6
        self.CONSTANT_ANGULAR_SPEED = 0.8
        self.CONSTANT_LINEAR_SPEED = 0.5

        self.max_range = float('inf')

        # Stores scan informatino of lidar (LaserScan.ranges)
        # From index 0 to 360, 0 is at x axis and 90 is at y axis
        self.current_scan = [1.0]   

        # Stores position of neato in odometry frame (Odometry.pose.pose.position)
        # We only care about x, y in this case
        self.current_position = None     
        
        # checks if the neato is bumping into something
        self.bumper_active = False

    def process_scan(self, msg):
        """
        Subscribes to LaserScan
        Get a series of distance from lidar.

        Vars:
            current_scan: a list of length 360, each is a distance
                @ angle (index)
            max_range: longest effective distance.
                If measurement is bigger than this, count as no object detected.
        """
        self.current_scan = msg.ranges
        self.max_range = msg.range_max

    def process_bump(self, msg):
        """Subcribes to bump msg:
            If any of the bumper sensor is True,
                set the bumper_active to True.
    
        """
        self.bumper_active = (msg.left_front == 1 or \
                              msg.left_side == 1 or \
                              msg.right_front ==1 or \
                              msg.right_side == 1)

    def check_odom (self,msg):
        """
        Subcribes to neato location and orientation
            in odometry frame.

        Vars:
            current_position: position in neato frame. 
                calls through self.current_position.x and self.current_position.y.
        """
        self.current_position = msg.pose.pose.position

    def get_coordinates(self,distance,index):
        """
        Converts from polar coordinate to cartesian coordinate,
            in respect to centered at neato.

        Args:
            index(int): angle in degree, represented as the index in laserscan list.
            distance (float):  distance in meter

        Returns:
            delta_x: displacement in x, in odometry
            delta_y: displacement in y, in odometry
        """
        index = math.radians(index)
        delta_x = distance*math.cos(index)
        delta_y = distance*math.sin(index)
        return delta_x,delta_y

    def calculate_center(self):
        """
        Calcultes the center of obstacles in respect to neato
            by finding average x, y from valid positions.

        Returns:
            sum_delta_x/cnt: average displacement x in respect to neato
            sum_delta_y/cnt: average displacement y in respect to neato
        """
        sum_delta_x = 0
        sum_delta_y = 0
        n = len(self.current_scan)
        cnt = 0
        for i in range(n):
            if  0 < self.current_scan[i] < self.max_range:
                # only valid scan is calculated
                dx,dy = self.get_coordinates(self.current_scan[i],i)
                sum_delta_x += dx
                sum_delta_y += dy
                cnt += 1
        if cnt == 0:
            return 0,0
        return sum_delta_x/cnt, sum_delta_y/cnt

    def send_msg(self):
        """
        Finds shortest distance from valid lidar scan and angle at that distance.
        
        """
        msg = Twist()
        dx,dy = self.calculate_center()
        if dx == 0:
            target_angle = 0
        else:
            target_angle = math.atan(dy/dx)
        target_distance = abs(math.sqrt(dy**2 + dx**2)-0.2)
        if dx < 0:
            # If the obstacle is behind us, turn first without going
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            
        else:
            msg.linear.x = max(self.kp * self.CONSTANT_LINEAR_SPEED * target_distance,0.05)
            msg.angular.z = max(self.kp*self.CONSTANT_ANGULAR_SPEED * target_angle, 0.05)
        print(msg.linear.x, msg.angular.z)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = person_follower() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()