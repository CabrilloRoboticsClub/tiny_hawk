#!/usr/bin/env python3
# Makes the file an executable

# Import the ROS Client Library for the Python language
import rclpy
# Also import Node 
from rclpy.node import Node

class Hello(Node): # Inherits from Node
    def __init__(self):
        super().__init__("Hello")  # Calls the constructor of the Node class
        self.create_timer(1.0, self.timer_callback)  # Calls timer_callback every 1.0 seconds
        self._counter = 0  # Initializes a counter so we can see how long our program has been running
    
    def timer_callback(self):
        self.get_logger().info(f"Hello world {self._counter}")  # Output `Hello world`
        self._counter += 1  # Increment the counter by 1 every 1 second

def main(args=None):
    rclpy.init(args=args) # Initialize communications 
    node = Hello()   # Instance
    rclpy.spin(node) # Keeps the node alive
    rclpy.shutdown() # Shutdown communications and destroy node

if __name__ == "__main__":
    main()