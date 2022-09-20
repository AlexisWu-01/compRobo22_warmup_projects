import drive_square
import obstacle_avoider
import person_follower
import teleop
import wall_follower
import rclpy




def main():
    while True:
        # start listening for a keyboard input
        controls = teleop.listen()

        # Tell the neato to start following an object
        if controls == "U":
            person_follower()

        # Tell the neato to start following a wall
        if controls == "I":
            wall_follower()

        # Tell the neato to start dodging obstacles
        if controls == "O":
            obstacle_avoider()

        # Tell to the neato to start driving in squares
        if controls == "P":
            drive_square()

        # exit the python program
        if controls == "M":
            # shut down ros
            rclpy.shutdown()
            # quit the python program
            quit()




if __name__ == "__main__":
    main()