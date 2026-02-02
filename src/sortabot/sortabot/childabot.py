import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random


class ChildABot(Node):
    def __init__(self):
        super().__init__("childabot")

        self.declare_parameter("bot_id", 0)
        bot_id = self.get_parameter("bot_id").value

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.scan_cb, 10)

        self.turn = 0.0
        self.timer = self.create_timer(0.15, self.step)

    def scan_cb(self, msg):
        if min(msg.ranges) < 0.6:
            self.turn = random.choice([-1.8, 1.8])

    def step(self):
        msg = Twist()
        msg.linear.x = 0.7
        msg.angular.z = self.turn + random.uniform(-0.3, 0.3)
        self.cmd_pub.publish(msg)
        self.turn *= 0.8


def main():
    rclpy.init()
    rclpy.spin(ChildABot())
    rclpy.shutdown()