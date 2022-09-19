import rclpy #importing ros
from rclpy.node import Node
from std_msgs.msg import String #datatype needed for ros to understand
from geometry_msgs.msg import Twist, Vector3 #for the neato
import time
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

        # set how many times to run send_message per second
        timer_period = 0.1
        # run send_message as many times per second as defined by timer_period
        self.timer = self.create_timer(timer_period,self.send_msg)

        # create a publisher that facilitates any movement (Twist) messages we send
        self.publisher = self.create_publisher(Twist,"cmd_vel",10)
        self.start_time = None

    def send_msg(self):
        # create a ros movement message
        straight = Twist()
        # set the forward velocity
        straight.linear.x = 0.1

        # create a ros movement message
        turn = Twist()
        # set the turning velocity
        turn.angular.z = 0.25

        # store initial time to measure time elapsed
        if self.start_time is None:
            self.start_time = time.time()

        # record the current time minus the start time for time elapsed
        elapsed = time.time() - self.start_time

        if elapsed < 10:
            # move forward for 10 seconds
            self.publisher.publish(straight)
        elif elapsed < 16.28:
            # turn for 6.28 seconds (90 degrees)
            self.publisher.publish(turn)
        else:
            # once the robot has moved forward + turned, reset timer
            self.start_time = time.time()



def main(args=None):
    # start up ros
    rclpy.init(args=args)

    # create an instance of the class (starts the publisher)
    node = drive_square()

    # tells ros to keep running and looping
    rclpy.spin(node)

    # allows ros to properly shutdown if code is exited
    rclpy.shutdown()

if __name__ == '__main__':
    main()