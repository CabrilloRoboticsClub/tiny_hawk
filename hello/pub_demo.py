#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String

class publisher_demo(Node):
    def __init__(self):
        super().__init__("publisher")
        self.get_logger().info("Publisher node started")
        self._publisher = self.create_publisher(String, "demo_topic", 10)
        self.create_timer(0.5, self.pub_callback) 
        self._counter = 0       
    
    def pub_callback(self):
        self._publisher.publish(f"your name, {self._counter}")
        self.get_logger().info(f"Message #{self._counter}: My name is ___")
        self._counter += 1

def main(args=None):
    rclpy.init(args=args)
    pub_node = publisher_demo()
    rclpy.spin(pub_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()