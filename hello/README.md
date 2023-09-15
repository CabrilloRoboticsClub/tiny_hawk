<h1 align="center">Hello World from ROS2</h1>
<h3 align="center">CRC</h3>

<br>
  <p align="center">
    GitHub Repo
    <br/>
    <a href="https://github.com/CabrilloRoboticsClub/tiny_hawk"><strong>Tiny Hawk »</strong></a>
    <br/><br/>
    <a href="https://app.theconstructsim.com/rosjects/my_rosjects">The Construct</a>
    ·
    <a href="https://github.com/CabrilloRoboticsClub">Cabrillo Robotics Club</a>
    ·
    <a href="https://docs.ros.org/en/foxy/index.html">ROS2 Docs</a>
  </p>
<br>


## Creating a package 
1. Navigate to the `src` directory with `cd ros2_ws/src/` 
2. Create a package with
   ```sh
   ros2 pkg create hello --build-type ament_python --dependencies rclpy
   # Where hello is the name of your package
   # Add dependencies (other packages your package relies on) after --dependencies
   ```
   A package is a subsystem to group together related executables. Packages are the most atomic unit of build and the unit of release
1. Build the package with `colcon build`
3. Open the new package in VS code called `hello`. There will be several pre-made folders and files. One has the same name as the package name, this is where we create the nodes
   
   Files you will see:
     - **pkg.xml**
        - The name of the package
        - The license 
        - Dependencies. Any time your package depends on another package, you add it to the dependencies 
        - Publication stuff 
      - **setup.py**
        - Similar to pkg.xml 
        - Where ROS2 installs nodes
      - **setup.cfg**
        - Do not touch
      - **pkg_name**
        - Directory with the same name as your package (`hello`)
        - This is where you create nodes 
        - There will be an `__init__.py` file by default 

<br>

## Hello world
1. Navigate to the package with `cd hello/hello/`
2. Create a new python file with `touch hello.py` to create a node
3. Make it executable with `chmod +x hello.py`
4. Open `hello.py` in VS Code
5. Add a shebang at the top of the file to make it executable 
    ```py
    #!/usr/bin/env python3
    ```
6. Import ROS Client Library for Python (`rclpy`) and from that `Node`
   ```py 
   import rclpy
   from rclpy.node import Node
   ```
7. Create a `main()` function
      ```py 
      def main(args=None):
        pass
      
      if __name__ == "__main__":
        main()
      ```
8. The first thing to do in the `main()` function is to initialize ROS2 communications and then terminate them
    ```py 
    rclpy.init(args=args) # Initialize communications 
    # Here is the node itself, it is not the file or program itself
    rclpy.shutdown() # Shutdown communications and destroy node
    ```
9. Nodes are created using Object Orientated Programming (OOP). Create a class that inherits from rclpy's Node, this allows us to have access to all the functionalities of ROS2
    ```py
    class Hello(Node):
      def __init__(self):
           super().__init__("Hello") # Calls the constructor of the Node class
    ```
10. Go back to `main()` and create an instance of the node
     ```py
     node = Hello()
     ```
11. Go back to the constructor and use the logger to output "Hello world". This is basically a print statement
    ```py
    self.get_logger().info("Hello world")
    ```
12. After creating the node and before the shutdown, call the `spin()` function. This keeps the node alive indefinitely until you kill it with `CTRL-C`. When the node is killed, the `spin()` function returns
      ```py
      rclpy.spin(node)
      ```
13. We would like to run the node with `ros2 run` and to do this we need to set up the node. We will do this in `setup.py`. Go inside the array of `console scripts` and add a line that states:
    ```py
    # executable = package.file_name:function_to_run
    "hello = hello.hello:main"
    ```
14. Navigate to the workspace directory. We can skip having to re-build the script after making changing the by entering the following in the terminal
    ```sh
    cd ../../..
    colcon build --symlink-install
    source ~/.bashrc
15. The previous command will set you back to the home directory, so we need to navigate back to the workspace to run the executable. Be sure to always run from the workspace
    ```sh
    cd ros2_ws/
    ros2 run hello hello
    ```
16. Use `CTRL-C` to kill the node
17. A very common task in ROS2 is to use a timer and a callback. This enables you to call a function every n seconds. We will be printing "Hello World" every second
    1.  First create a timer as another function in the class 
        ```py 
        def timer_callback(self):
          self.get_logger().info("Hello world")
        ```
    2. This will replace the similar statement in the `__init__()` function, so you can remove it
    3. Within the `__init_()` function create a timer that calls the `timer_callback()` function such that it repeats every second
        ```py 
        # Calls timer_callback every 1.0 seconds
        self.create_timer(1.0, self.timer_callback)
        ```
    4. Add a class timer variable to keep track of the number of seconds passed 
        ```py 
        self._counter = 0
        ```
    5. We can output the `_counter`'s value in our callback function by updating the logger to output an f string
        ```py
        self.get_logger().info(f"Hello world {self._counter}")
        ```
    6. Finally ensure the `_counter` updates by incrementing it in the callback
        ```
        self._counter += 1
        ```
18. Run your node again to view the changes
    ```sh
    ros2 run hello hello
    ```
19. Congratulations, you have created your first node in ROS2


<br>

## ROS2 topics 
### What is a topic
A topic is a communication channel for nodes to publish (send) and subscribe (receive) information from. This is one of the ways to move information between nodes. You can have many publishers and many subscribers to topics
<br>

### demo_nodes_cpp
1. ROS has several built in examples, one of which is demo_nodes_cpp. We will look at this to explore what topics are
2. Start the `talker` (publisher) with 
    ```sh
    ros2 run demo_nodes_cpp talker
    ```
    You should see Hello World and a number 
1. Now start a new terminal and start the `listener`
    ```sh
    ros2 run demo_node_cpp listener
    ```
    You should notice the number attached to Hello World here is the same as from the `talker`
1. Open a new terminal and enter the command `rqt_graph`
2. This is an example of publisher and subscriber communication, the nodes communicate using topics, in this example we are using the topic `chatter`. You can see the available topics with 
    ```sh
    ros2 topic list
    ```
3. To see more information the topic including how many nodes are publishing/subscribing to it and the type of data it uses, use the command
    ```sh 
    # ros2 topic info /topic_name
    ros2 topic info /chatter # Where chatter is the name of the topic
    ```
4. To read what is being sent over the topic use 
   ```sh 
   # ros2 topic echo /topic_name
   ros2 topic echo /chatter # Where chatter is the name of the topic
   ```
    This actually creates another subscriber