import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller", namespace="sortabot")

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "scan", self.scan_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.front_clear = True
        self.target = (10.0, 10.0)

        self.timer = self.create_timer(0.1, self.step)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def scan_cb(self, msg):
        mid = len(msg.ranges) // 2
        window = msg.ranges[mid-20:mid+20]
        self.front_clear = all(r > 0.8 or math.isinf(r) for r in window)

    def step(self):
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        dist = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        err = math.atan2(math.sin(ang - self.yaw), math.cos(ang - self.yaw))

        cmd = Twist()

        if not self.front_clear:
            cmd.angular.z = 0.8
        elif abs(err) > 0.2:
            cmd.angular.z = err
        elif dist > 0.3:
            cmd.linear.x = 0.6

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()