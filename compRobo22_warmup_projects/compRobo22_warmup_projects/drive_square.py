import rclpy #importing ros
# import rospy #using ros time
from rclpy.node import Node
from std_msgs.msg import String #datatype needed for ros to understand
from geometry_msgs.msg import Twist, Vector3 #for the neato
import time

class drive_square(Node):
    def __init__(self):
        super().__init__("publisher_node") #call parent class

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.send_msg) #runs at timer_period, then call the call back function
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)#name of topic,"cmd_vel" for moving; queue size

    def send_msg(self):
        # record the current time
        prev = time.time()


        # msg = String(data = 'hello') #each msg has a label to go with, String is a datatype
        # msg = String()
        # msg.data = 'hello'
        straight = Twist(linear=Vector3(x=1.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        turn = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=-0.5))
        #msg = Twist
        #msg.lin_vel =

        while (time.time()-prev < 1):
            self.publisher.publish(straight)
        prev = time.time()
        while (time.time()-prev < 3.14):
            self.publisher.publish(turn)



def main(args=None):
    rclpy.init(args=args) #initializing ros ?lookup
    node = drive_square() #node instance
    rclpy.spin(node)#like a while loop, can be done from other things
    rclpy.shutdown()

if __name__ == '__main__':
    main()