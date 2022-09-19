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
    def __init__(self):
        super().__init__("publisher_node") #call parent class
    
        timer_period = 0.05
        self.create_subscription(LaserScan,'scan',self.process_scan,10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(Odometry,"odom",self.check_odom,10 )
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)#name of topic,"cmd_vel" for moving; queue size
        self.timer = self.create_timer(timer_period,self.send_msg) #runs at timer_period, then call the call back function
        self.current_scan = [1.0]
        self.kp = 0.6
        self.CONSTANT_ANGULAR_SPEED = 0.8
        self.CONSTANT_LINEAR_SPEED = 0.3

        self.max_range = float('inf')
        self.current_position = None
        self.bumper_active = False
        self.callibration = False
        self.to_turn = False
        self.to_go = True
    

    def process_scan(self, msg):
        self.current_scan = msg.ranges
        self.max_range = msg.range_max
        # print(self.current_scan)

    def process_bump(self, msg):
        self.bumper_active = (msg.left_front == 1 or \
                              msg.left_side == 1 or \
                              msg.right_front ==1 or \
                              msg.right_side == 1)

    def check_odom (self,msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = euler_from_quaternion(msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w)

    def get_coordinates(self,distance,index):
        index = math.radians(index)
        delta_x = distance*math.cos(index)
        delta_y = distance*math.sin(index)
        return delta_x,delta_y

    def calculate_center(self):
        sum_delta_x = 0
        sum_delta_y = 0
        n = len(self.current_scan)
        cnt = 0
        for i in range(n):
            if  0 < self.current_scan[i] < self.max_range:
                dx,dy = self.get_coordinates(self.current_scan[i],i)
                sum_delta_x += dx
                sum_delta_y += dy
                cnt += 1
        if cnt == 0:
            return 0,0
        return sum_delta_x/cnt, sum_delta_y/cnt




    def send_msg(self):
        msg = Twist()
        dx,dy = self.calculate_center()
        if dx == 0:
            target_angle = 0
        else:
            target_angle = math.atan(dy/dx)
        target_distance = math.sqrt(dy**2 + dx**2)
        # print(dx,dy)
        # print(target_angle,target_distance)
        if dx < 0:
            msg.linear.x = 0.0
        else:
            msg.linear.x = self.kp * self.CONSTANT_LINEAR_SPEED * target_distance
        msg.angular.z = self.kp*self.CONSTANT_ANGULAR_SPEED * target_angle
        print(msg.linear.x,msg.angular.z)
        self.publisher.publish(msg)
            

               



def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = person_follower() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()