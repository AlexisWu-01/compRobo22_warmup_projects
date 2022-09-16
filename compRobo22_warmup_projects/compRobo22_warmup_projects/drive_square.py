import rclpy #importing ros
from rclpy.node import Node
from std_msgs.msg import String #datatype needed for ros to understand
from geometry_msgs.msg import Twist, Vector3 #for the neato
import time
from neato2_interfaces.msg import Bump

class drive_square(Node):
    def __init__(self):
        super().__init__("publisher_node") #call parent class

        timer_period = 0.05
        self.timer = self.create_timer(timer_period,self.send_msg) #runs at timer_period, then call the call back function
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)#name of topic,"cmd_vel" for moving; queue size
        self.start_time = None
        self.bumper_active = False
        

    def send_msg(self):
        straight = Twist()
        straight.linear.x = 0.1
        turn = Twist()
        turn.angular.z = 0.25
        stop = Twist()
        
        if self.bumper_active:
            self.publisher.publish(stop)
            self.start_time = None
        else:
            if self.start_time is None:
                self.start_time = time.time()
            # record the current time
            elapsed = time.time() - self.start_time
            if elapsed < 10:
                self.publisher.publish(straight)
            # elif elapsed < 2.2:
            #     self.publisher.publish(stop)
            elif elapsed < 16.28318:
                self.publisher.publish(turn)
            else:
                self.start_time = time.time()
            
            

            # msg = String(data = 'hello') #each msg has a label to go with, String is a datatype
            # msg = String()
            # msg.data = 'hello'

            #straight = Twist(linear=Vector3(x=1.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
            #turn = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=-0.5))
            #msg = Twist
            #msg.lin_vel =

            #while (time.time()-prev < 1):
            #    self.publisher.publish(straight)
            #prev = time.time()
            #while (time.time()-prev < 3.14):
            #    self.publisher.publish(turn)

    def process_bump(self, msg):
        self.bumper_active = (msg.left_front == 1 or \
                              msg.left_side == 1 or \
                              msg.right_front ==1 or \
                              msg.right_side == 1)
        print(self.bumper_active)

def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = drive_square() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()