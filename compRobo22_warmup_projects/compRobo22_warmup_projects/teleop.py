import rclpy #importing ros
from rclpy.node import Node
from getch import getch
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class teleop(Node):
    """
    A class used to present keyboard commands of a Neato robot,
    and read the terminal keyboard input of the user.

    ...

    Methods
    -------
    load_screen()
        Prints the keyboard commands for operating the Neato robot.
    listen()
        Reads keyboard input from the terminal, prints the last input,
        and returns the input converted to uppercase.
    """

    def __init__(self):
        """
        Runs the load_screen method on startup

        Args:

        Returns:
        """
        super().__init__("teleop")

        # print the load screen
        self.load_screen()

        # initialize a publisher for any key strokes.
        self.keys = self.create_publisher(String, 'Keys', 10)

        # initializer a publisher for basic keystroke movement
        self.movement = self.create_publisher(Twist, 'cmd_vel', 10)

        # run the run_loop function
        self.create_timer(0.1, self.run_loop)

        # set the angular velocity
        self.angular_velocity = .2

        # set the linear velocity
        self.linear_velocity = .25

    def run_loop(self):

        # record the keypress
        key_press = self.listen()

        # make sure there is a keypress
        if key_press is not None:

            # create an empty ros string message
            msg = String()

            # populate the message with the keypress
            msg.data = key_press

            # publish the message through the Keys publisher
            self.keys.publish(msg)

            if key_press == "W":
                self.neato_forward()
            elif key_press == "Q":
                self.turn_left()
            elif key_press == "E":
                self.turn_right()
            elif key_press == "A":
                self.neato_stop()
            elif key_press == "D":
                self.neato_stop()
            elif key_press == "S":
                self.neato_backward()
            elif key_press == "M":
                quit()
        else:
            return

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

    def neato_stop(self):
        """
        Tells the neato to stop moving.

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()
        # send the message through the publisher
        self.movement.publish(msg)

    def neato_forward(self):
        """
        Tells the neato to move forward

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()

        # set the linear velocity
        msg.linear.x = self.linear_velocity

        # send the message through the publisher
        self.movement.publish(msg)

    def neato_backward(self):
        """
        Tells the neato to move backward

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()

        # set the linear velocity
        msg.linear.x = -self.linear_velocity

        # send the message through the publisher
        self.movement.publish(msg)

    def load_screen(self):
        """
        Prints a guide of keyboard commands for the Neato.

        Args:

        Returns:
        """

        # create the initial loading screen
        print("_____Controls_____\n" +
               "        ^         |U Person Follower  \n" +
               "  <\ Q  W  E />   |I Wall Follower    \n" +
               "    X  A D  X     |O Obstacle Avoider \n" +
               "        S         |P Drive Square     \n" +
               "        v         |M Escape             ")

    def listen(self):
        """
        Listens for a key input from the user, converts the input to uppercase,
        and prints the last keypress in the terminal.

        Args:

        Returns:
            key_hit: an uppercase string representing the key pressed
            by the user in the terminal.
        """

        # listen for the a key hit
        key_hit = getch()

        # convert any input to uppercase
        key_hit = key_hit.upper()

        # print the keystroke while deleting the last one
        sys.stdout.write('\r' + "       {" + key_hit + "}       ")

        return key_hit


def main(args=None):
    rclpy.init(args=args)
    node = teleop()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()