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
    def __init__(self):
        super().__init__("publisher_node") #call parent class
    
        timer_period = 0.05
        self.create_subscription(LaserScan,'scan',self.process_scan,10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(Odometry,"odom",self.check_odom,10 )
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)#name of topic,"cmd_vel" for moving; queue size
        self.timer = self.create_timer(timer_period,self.send_msg) #runs at timer_period, then call the call back function
        self.current_scan = []
        self.kp = 0.4
        self.CONSTANT_ANGULAR_SPEED = 0.5
        self.CONSTANT_LINEAR_SPEED = 0.5

        self.shortest_angle = None
        self.shortest_distance = float('inf')
        self.max_range = float('inf')
        self.current_position = None
        self.bumper_active = False
        self.callibration = False
        self.to_turn = True
        self.to_go = False
        self.target_angle = None
        self.target_distance = None
        self.go_parallel = False
        self.turn_vertical = False

        

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
        delta_x = distance*math.cos(index)
        delta_y = distance*math.sin(index)
        return delta_x,delta_y

    def find_best_angle(self):
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
                        self.shortest_distance = shortest_distance - .3
                        self.shortest_angle = math.radians(shortest_angle)
                        break
                        #at least 90 consecutive valid ranges should be there in case it is not a wall (meaning the neato should be within 1.8m to the wall to be detected)
                    else:
                        sum_delta_x, sum_delta_y, cnt = 0, 0, 0
                        shortest_distance = float('inf')
                        shortest_angle = None
           
        except:
            pass

    def send_msg(self):
        msg = Twist()
        if not self.shortest_angle:
            self.find_best_angle()
            if not self.shortest_angle:
                print('no wall detected!')
        else:
            if self.to_turn:
                if not self.target_angle:
                    self.target_angle = self.shortest_angle + self.current_orientation[2]
                if self.turn_vertical:
                    self.target_angle = self.current_orientation[2] + math.pi/2
                    self.go_parallel = True
                    self.turn_vertical = False
                    print('check here:',self.target_angle - self.current_orientation[2])
                angle_to_turn = self.target_angle - self.current_orientation[2]
                print('angle to turn:',angle_to_turn)
                if abs(angle_to_turn) < 1e-2:
                    # already facing best angle
                    msg.angular.z = 0.0
                    self.to_go = True
                    self.to_turn = False
                else:
                    msg.angular.z = self.kp*self.CONSTANT_ANGULAR_SPEED * angle_to_turn

            if self.to_go:
                if self.go_parallel:
                    msg.linear.x = self.CONSTANT_LINEAR_SPEED
                    if self.current_scan[270] > self.max_range:
                        msg.linear.x = 0.0
                else:
                    if not self.target_distance:
                        dx,dy = self.get_coordinates(self.shortest_distance,self.shortest_angle)
                        self.target_distance = [dx+self.current_position.x, dy+self.current_position.y]
                    distance_to_go = math.sqrt((self.target_distance[0]-self.current_position.x)**2+(self.target_distance[1]-self.current_position.y)**2)
                    print('distance_to_go:',distance_to_go)

                    if (distance_to_go) < 3e-2:
                        msg.linear.x = 0.0
                        self.to_go = False
                        # self.to_turn = True
                        self.to_turn = True
                        self.turn_vertical = True

                    else:
                        msg.linear.x = self.kp * self.CONSTANT_LINEAR_SPEED * distance_to_go
            
        # print(self.shortest_angle,self.shortest_distance)
        self.publisher.publish(msg)


        

def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = wall_follower() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            