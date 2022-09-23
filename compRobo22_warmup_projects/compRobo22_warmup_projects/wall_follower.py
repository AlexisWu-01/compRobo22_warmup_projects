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


class wall_follower(Node):

    """
    A ros node to make the Neato finds a wall and follows it.

    ...

    Methods
    -------
    send_msg()
        Loops through the 5 patterns: 
        1. finds wall: finds shortest distance from lidar scan after filtering
        2. to_turn: from the best shortest lidar scan, turn the neato to vertically towards wall.
        3. to_go: approaches wall until reaches a close enough distance from wall.
        4. turn_vertical: same with to_turn, but target angle is 90 degree, not calculate from trigonometry.
        5. go_parallel: go parallel with wall, stops running when the wall is no longer detected.

    """
    def __init__(self):
        super().__init__("publisher_node") 

        timer_period = 0.05 # the time at which send_msg() is called

        #Subscribe to laser scan: gets a list of distance from 360 degree laser scan
        self.create_subscription(LaserScan,'scan',self.process_scan,10)

        # Subscribe to bumper sensor: the neato would not go any further if it hits an obstacle
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        # Subscribe to position and orientation information from odometry
        self.create_subscription(Odometry,"odom",self.check_odom,10 )

        # Publish msg to neato, which is speed in this publisher
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)

        # Runs send_msg at the defined timer_period
        self.timer = self.create_timer(timer_period,self.send_msg)



        # Constant coefficients for proportional control
        self.kp = 0.5
        self.CONSTANT_ANGULAR_SPEED = 0.3
        self.CONSTANT_LINEAR_SPEED = 0.5

        # Stores scan informatino of lidar (LaserScan.ranges)
        # From index 0 to 360, 0 is at x axis and 90 is at y axis
        self.current_scan = []

        # Stores position of neato in odometry frame (Odometry.pose.pose.position)
        # We only care about x, y in this case
        self.current_position = None

        # Stores orientation of neato in odometry frame, centered as neato center
        # We only care about z in this case, which is [2] in the list
        self.current_orientation = None

        # shortest angle from laserscan to turn towards wall
        self.shortest_angle = None
        # shortest distance from laserscan
        self.shortest_distance = float('inf')

        # checks if the neato is bumping into something
        self.bumper_active = False

        # state machines to determine neato's move pattern
        self.to_turn = True
        self.to_go = False
        self.go_parallel = False
        self.turn_vertical = False

        #target position in odometry frame
        self.target_distance = None

        #target angle in odometry frame
        self.target_angle = None

        self.max_range = float('inf')


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
            current_orientation: orientation in neato frame.
                transformed to neato center in odometry frame.
                class through self.current_orientation[2] (which is z).

        """
        self.current_position = msg.pose.pose.position
        self.current_orientation = euler_from_quaternion(msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w)



    def get_coordinates(self,distance,angle):
        """
        Converts from polar coordinate to cartesian coordinate,
            in respect to centered at neato.

        Args:
            distance (float):  distance in meter
            angle (float): angle in radians

        Returns:
            delta_x: displacement in x, in odometry
            delta_y: displacement in y, in odometry
        """
        delta_x = distance*math.cos(angle)
        delta_y = distance*math.sin(angle)
        return delta_x,delta_y

    def find_best_angle(self):
        """
        Finds shortest distance from valid lidar scan and angle at that distance.
        
        """
        try:
            n = len(self.current_scan)
            cnt = 0
            shortest_distance = self.shortest_distance
            shortest_angle = self.shortest_angle
            for i in range(n):
                if  0 < self.current_scan[i] < self.max_range:
                    cnt += 1
                    if shortest_distance > self.current_scan[i]:
                        shortest_distance = self.current_scan[i]
                        shortest_angle = i
                else:
                    if cnt  >= 90: 
                        #at least 90 consecutive valid ranges to count as a wall
                        # to prevent noise and other obstacles.
                        self.shortest_distance = shortest_distance - .3
                        self.shortest_angle = math.radians(shortest_angle)
                        break
                    else:
                        sum_delta_x, sum_delta_y, cnt = 0, 0, 0
                        shortest_distance = float('inf')
                        shortest_angle = None
           
        except:
            pass

    def send_msg(self):
        """
        Publishes velocity information in each timer_period
        """
        if not self.bumper_active:
            msg = Twist()

            if not self.shortest_angle:
                # Only need to locate target angle once.
                self.find_best_angle()
                if not self.shortest_angle:
                    print('no wall detected!')
            else:
                if self.to_turn:
                    # turns angle at some degree
                    if not self.target_angle:
                        # initialize target_angle only once
                        self.target_angle = self.shortest_angle + self.current_orientation[2]
                    if self.turn_vertical:
                        # if state is turn_vertical, set target_angle = 90 degree
                        self.target_angle = self.current_orientation[2] + math.pi/2
                        self.go_parallel = True
                        self.turn_vertical = False

                    angle_to_turn = self.target_angle - self.current_orientation[2]
                    print('angle to turn:',angle_to_turn)
                    if abs(angle_to_turn) < 1e-2:
                        # already facing best angle
                        msg.angular.z = 0.0
                        self.to_go = True
                        self.to_turn = False
                    else:
                        msg.angular.z = max(self.kp*self.CONSTANT_ANGULAR_SPEED * angle_to_turn,0.03)

                if self.to_go:
                    #goes straight at a certain distance
                    if self.go_parallel:
                        #if state set to go_parallel, follow the wall at a constant speed.
                        msg.linear.x = self.CONSTANT_LINEAR_SPEED*self.kp
                        if self.current_scan[270] > self.max_range:
                            # if no wall no the right detected, stop neato.
                            msg.linear.x = 0.0
                    else:
                        # approach towards wall
                        if not self.target_distance:
                            # initialize target_distance only once.
                            dx,dy = self.get_coordinates(self.shortest_distance,self.shortest_angle)
                            self.target_distance = [dx+self.current_position.x, dy+self.current_position.y]
                        distance_to_go = math.sqrt((self.target_distance[0]-self.current_position.x)**2+(self.target_distance[1]-self.current_position.y)**2)
                        print('distance_to_go:',distance_to_go)

                        if (distance_to_go) < 3e-2:
                            # reached target distance
                            msg.linear.x = 0.0
                            self.to_go = False
                            self.to_turn = True
                            self.turn_vertical = True

                        else:
                            msg.linear.x = max(self.kp * self.CONSTANT_LINEAR_SPEED * distance_to_go,0.05)
                
            self.publisher.publish(msg)


        

def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = wall_follower() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            