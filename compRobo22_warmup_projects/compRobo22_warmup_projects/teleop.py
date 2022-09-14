from getch import getch
import sys



class teleop():
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
        self.load_screen()



    def load_screen(self):
        """
        Prints a guide of keyboard commands for the Neato.

        Args:

        Returns:
        """

        # create the initial loading screen
        print("_____Controls_____\n" +
               "        ^         \n" +
               "  <\ Q  W  E />   \n" +
               "    <  A D  >     \n" +
               "        S         \n" +
               "        v         ")

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


def main():
    run = teleop()

    # listen forever for key input
    while True:
        # start listening for keystrokes
        key_hit = run.listen()



if __name__ == '__main__':
    main()