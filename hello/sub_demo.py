#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String

class subscriber_demo(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.get_logger().info("Subscriber node started")
        self.subscription = self.create_subscription(String,'demo_topic', self.sub_callback, 10)
    
    def sub_callback(self, msg):
        self.get_logger().info(f"Hello {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    sub_node = subscriber_demo()
    rclpy.spin(sub_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()