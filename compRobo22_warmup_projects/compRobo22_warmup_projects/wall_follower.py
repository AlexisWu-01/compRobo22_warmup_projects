import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('distance_emergency_stop')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_to_obstacle = None
        self.Kp = 0.4
        self.target_distance = 0.5
        self.angular_vel = 0.1
        self.linear_vel = 0.1
        self.parallel = False
        self.start_time = None
        self.approach = False
        self.turn_square = False
        self.x_axis = -1
        self.y_axis = -1
        self.max_range = -1
        self.cur_x = None
        self.cur_y = None
        self.found = False

    
    

    # def run_loop(self):
    #     msg = Twist()
    #     if self.distance_to_obstacle is None:
    #         msg.linear.x = 0.1
    #     else:
    #         msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)
    #     self.vel_pub.publish(msg)

    def stop_neato(self):
        #sets all neato velocities to 0
        msg = Twist()
        self.vel_pub.publish(msg)

    def turn_neoto(self):
        #ask neato to turn
        msg = Twist()
        msg.angular.z = self.angular_vel # future improvement: add proportional speed
        self.vel_pub.publish(msg)
            
    def go_straight(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        self.vel_pub.publish(msg)


    def process_scan(self, msg):
        self.x_axis = msg.ranges[0]
        self.y_axis = msg.ranges[90]
        self.max_range = msg.range_max
   
    
    def approach_wall(self):
            if self.x_axis == self.target_distance:
                self.turn_square = True
            msg = Twist()
            msg.linear.x = self.Kp*(self.x_axis - self.target_distance)
            self.vel_pub.publish(msg)

    def run_loop(self):
        if self.parallel:
            #goal achieved, just follow the line
            print('following wall!')
            self.go_straight()

        elif not self.found and (self.x_axis > self.max_range or self.y_axis > self.max_range):
            # one of x and y axis is not facing the wall
            #turn towards wall
            print('finding wall')
            self.turn_neoto()

        elif self.approach:
            print('going near the wall')
            self.approach_wall()
        elif self.turn_square:
            print('turns parallel')
            if self.start_time is None:
                self.start_time = time.time()
            elapse = time.time()-self.start_time
            if elapse < 16.708:
                self.turn_neoto()
            else:
                self.parallel = True

        elif self.x_axis < self.max_range and self.y_axis < self.max_range:
            print('turning vertical')
            self.found = True
            if not self.cur_x:
                self.cur_x = self.x_axis
                self.cur_y = self.y_axis
            target_angle = math.atan(self.cur_x/self.cur_y)
            #turn vertical to wall
            turn_time = target_angle/self.angular_vel
            if self.start_time is None:
                self.start_time = time.time()
            elapse = time.time()- self.start_time
            if elapse < turn_time:
                self.turn_neoto()
            else:
                print('perfect angle!')
                self.approach = True
                self.start_time = None



        



def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()