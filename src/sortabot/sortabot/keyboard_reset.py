import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import sys
import termios
import tty


class KeyboardReset(Node):
    def __init__(self):
        super().__init__("keyboard_reset")
        self.cli = self.create_client(Empty, "reset_task")
        self.get_logger().info("Press 'r' to reset task")

    def run(self):
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                if sys.stdin.read(1).lower() == "r":
                    self.cli.call_async(Empty.Request())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    KeyboardReset().run()
    rclpy.shutdown()