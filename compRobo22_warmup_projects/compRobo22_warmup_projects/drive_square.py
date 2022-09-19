from turtle import position
import rclpy #importing ros
from rclpy.node import Node
from std_msgs.msg import String #datatype needed for ros to understand
from geometry_msgs.msg import Twist, Vector3 #for the neato
from neato2_interfaces.msg import Bump
from nav_msgs.msg import Odometry
from .angle_helpers import euler_from_quaternion
import math


class drive_square(Node):
    """
    A class used to make the Neato drive in a square

    ...

    Methods
    -------
    send_msg()
        Loops through the movements and turning necessary to move
        the Neato in a square. Uses the ros publisher defined
        in the init to send movement messages to the Neato.

    """

    def __init__(self):
        super().__init__("publisher_node") #call parent class

        timer_period = 0.05
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(Odometry,"odom",self.check_odom,10 )
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)#name of topic,"cmd_vel" for moving; queue size
        self.timer = self.create_timer(timer_period,self.send_msg) #runs at timer_period, then call the call back function

        self.kp = 0.6
        self.CONSTANT_ANGULAR_SPEED = 0.5
        self.CONSTANT_LINEAR_SPEED = 0.3

        self.current_position = None
        self.bumper_active = False
        self.callibration = False
        self.deltas = [(0.0,0.0),(1.0,0.0),(1.0,1.0),(0.0,1.0)]
        self.corners = [None,None,None,None]
        self.count = 1 #track which corner to go to next
        self.turn_angle = math.pi/2
        self.target_angle = None
        self.to_turn = False
        self.to_go = True


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

    def send_msg(self):
        if not self.callibration:
            #callibrates the start odom frame (real_world)
            print('start callibration')
            try:
                if self.count < 1:
                    pointer = 3
                else:
                    pointer = self.count - 1
                cur_x = self.current_position.x
                cur_y = self.current_position.y
                for i in range(len(self.deltas)):
                    self.corners[i] = (self.deltas[i][0]+cur_x-self.deltas[pointer][0],self.deltas[i][1]+cur_y-self.deltas[pointer][1])

                print(self.corners)
                print(self.current_position.x, self.current_position.y)
                self.callibration = True
            except:
                pass
        msg = Twist()
        try:
            error_x = self.corners[self.count][0] - self.current_position.x
            error_y = self.corners[self.count][1] - self.current_position.y
            error_distance = math.sqrt(error_x**2+error_y**2)
            if self.to_turn:
                if not self.target_angle:
                    self.target_angle = self.turn_angle + self.current_orientation[2]
                angle_to_turn = self.target_angle - self.current_orientation[2]
                if angle_to_turn > 5e-2:
                    msg.angular.z = self.kp * angle_to_turn * self.CONSTANT_ANGULAR_SPEED
                    print('turning at',msg.angular.z)
                    print(angle_to_turn,"to turn")
                else:
                    msg.angular.z = 0.0
                    self.to_go = True
                    self.to_turn = False
                    self.target_angle = None
                    self.callibration = False


            elif self.to_go:
                msg.linear.x = self.kp*self.CONSTANT_LINEAR_SPEED*error_distance
                print('going at',msg.linear.x)
                print(error_distance,"to go")
                if error_distance < 5e-2:
                    msg.linear.x = 0.0
                    self.to_turn = True
                    self.to_go = False
                    self.target_angle = None
                    print('head to next point')
                    if self.count < 3:
                        self.count += 1
                    else:
                        self.count = 0


        except:
            pass
        self.publisher.publish(msg)








    # tells ros to keep running and looping
    rclpy.spin(node)

    # allows ros to properly shutdown if code is exited
    rclpy.shutdown()

if __name__ == '__main__':
    main()