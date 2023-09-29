<h1 align="center">Hello World from ROS2</h1>
<h3 align="center">CRC Software Team</h3>

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

   <img width="208" alt="files" src="https://i.imgur.com/iXtHA1G.png">

   Notable files and directories:
     - **pkg.xml**
        - Defines the name of the package
        - Determines the license for the code 
        - Handles dependencies. Any time your package depends on another package, you add it to the dependencies 
        - Publication stuff 
      - **setup.py**
        - Similar to pkg.xml 
        - Where ROS2 installs nodes
      - **setup.cfg**
        - Do not touch
      - **pkg_name**
        - Directory with the same name as your package (`hello`)
        - This is where you will write code and create nodes
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
    rclpy.shutdown() # Shutdown communications and destroy node
    ```
9. Nodes are exist in your program as objects. Create a class that inherits from rclpy's Node, this allows us to have access to all the functionalities of ROS2. More info on Objects/Classes: Objects are data types that we (the developers) define, and they contain a combination of data as well as functions/methods. Objects are instances of a Class, which define the types of variables an object can have and how its functions work.
    ```py
    class Hello(Node):
      def __init__(self):
           super().__init__("Hello") # Calls the constructor of the Node class
    ```
10. Go back to `main()` and create an instance of the node
     ```py
     node = Hello()
     ```
11. Go back to the constructor function for the Hello Node and use the `get_logger()` function to output "Hello world" to the console, similar to `print("Hello world")`,`cout << "Hello World";`, and `System.out.println("Hello world");`. Note on constructors: since we define Classes ourselves, we can also create constructor methods to define how instances of the class should be created (e.g. setting member variables).
    ```py
    self.get_logger().info("Hello world")
    ```
12. After creating the node and before the shutdown, call the `spin()` function. This keeps the node alive indefinitely until you kill it with `CTRL-C`. When the node is killed, the `spin()` function returns
      ```py
      rclpy.spin(node)
      ```
13. Your code should now look like this:
    ```py
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node

    class Hello(Node):
      def __init__(self):
        super().__init__("Hello") # Calls the constructor of the Node class
        self.get_logger().info("Hello world")

    def main(args=None):
      rclpy.init(args=args) # Initialize communications 
      node = Hello()
      rclpy.spin(node)
      rclpy.shutdown() # Shutdown communications and destroy node

    if __name__ == "__main__":
      main()
    ```
14. We would like to run the node with `ros2 run` and to do this we need to set up the node. We will do this in `setup.py`. Go inside the array of `console scripts` and add a line that states:
    ```py
    # executable = package.file_name:function_to_run
    "hello = hello.hello:main"
    ```
15. Navigate to the workspace directory. We can skip having to re-build the script after making changing the by entering the following in the terminal
    ```sh
    cd ../../..
    colcon build --symlink-install
    source ~/.bashrc
16. The previous command will set you back to the home directory, so we need to navigate back to the workspace to run the executable. Be sure to always run from the workspace
    ```sh
    cd ros2_ws/
    ros2 run hello hello
    ```
17. Use `CTRL-C` to kill the node
18. A very common task in ROS2 is to use a timer and a callback. This enables you to call a function every n seconds. We will be printing "Hello World" every second
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
19. Run your node again to view the changes
    ```sh
    ros2 run hello hello
    ```
    <img width="650" alt="hello" src="https://i.imgur.com/pO6prot.png">

20. Congratulations, you have created your first node in ROS2
21. The full complete solution is available [here](https://github.com/CabrilloRoboticsClub/tiny_hawk/blob/main/hello/hello.py)


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
   
    <img width="650" alt="talker" src="https://i.imgur.com/QeAWupG.png">

1. Now start a new terminal and start the `listener`
    ```sh
    ros2 run demo_node_cpp listener
    ```
    You should notice the number attached to Hello World here is the same as from the `talker`
   
   <img width="650" alt="listener" src="https://i.imgur.com/6Hw8V3q.png">

1. Open a new terminal and enter the command `rqt_graph`. Then click on the the Graphical Tools icon on the toolbar to open the display
    ```sh
    rqt_graph 
    ```
   
    <img width="650" alt="rqt_graph" src="https://i.imgur.com/iPSwWPG.png">

3. This is an example of publisher and subscriber communication, the nodes communicate using topics, in this example we are using the topic `chatter`. You can see the available topics with 
    ```sh
    ros2 topic list
    ```
4. To see more information the topic including how many nodes are publishing/subscribing to it and the type of data it uses, use the command
    ```sh 
    # ros2 topic info /topic_name
    ros2 topic info /chatter # Where chatter is the name of the topic
    ```
5. To read what is being sent over the topic use 
   ```sh 
   # ros2 topic echo /topic_name
   ros2 topic echo /chatter # Where chatter is the name of the topic
   ```
    This actually creates another subscriber

<br>

## ROS2 pub/sub 
### What are publishers and subscribers 
In ROS2, publishers and subscribers are one of the ways in which nodes communicate with each other. These communications are passed over channels called topics as discussed above. A subscriber sends a message over a topic, and and a subscriber node can listen to that topic to receive data. In the previous example, the `talker` is a publisher and `listener` is a subscriber.

### Create a new package for the following demo
Navigate to the `src` directory with `cd ~/ros2_ws/src/` 
1. Create a package with
   ```sh
   ros2 pkg create pub_sub --build-type ament_python --dependencies rclpy
   ```
2. Build the package with `colcon build`
3. Open the new package in VS code called `pub_sub`


### Writing a publisher and subscriber
#### General setup
1. Navigate to the package with `cd pub_sub/pub_sub/`
2. Create files
   1. Create a file for the subscriber and make it executable
      ```sh
       touch sub_demo.py
       chmod +x sub_demo.py
      ```
   2. Create a file for the publisher and make it executable
       ```sh 
       touch pub_demo.py 
       chmod +x pub_demo.py
       ```
3. Add base code  
   1. In `sub_demo.py` add the basic framework for a ROS2 node
      ```py
      #!/usr/bin/env python3
      import rclpy 
      from rclpy.node import Node 

      class subscriber_demo(Node):
        def __init__(self):
          super().__init__("subscriber")
          self.get_logger().info("Subscriber node started")

      def main(args=None):
        rclpy.init(args=args)
        sub_node = subscriber_demo()
        rclpy.spin(sub_node)
        rclpy.shutdown()
      
      if __name__ == "__main__":
        main()
      ```
   2. In `pub_demo.py` add the basic framework for a ROS2 node 
      ```py
      #!/usr/bin/env python3
      import rclpy 
      from rclpy.node import Node 

      class publisher_demo(Node):
        def __init__(self):
          super().__init__("publisher")
          self.get_logger().info("Publisher node started")

      def main(args=None):
        rclpy.init(args=args)
        pub_node = publisher_demo()
        rclpy.spin(pub_node)
        rclpy.shutdown()
      
      if __name__ == "__main__":
        main()
      ```
4. Set up the nodes in `setup.py` by adding the following to the `console_scripts` value list
    ```py
    "sub_demo = pub_sub.sub_demo:main",
    "pub_demo = pub_sub.pub_demo:main"
    ```
    Make sure the list items are comma separated 

#### Publisher:
1. Add a timer to the publisher's constructor. The timer specifies how frequently the nodes will publish messages
    1. Initialize the timer 
        ```py
        self.create_timer(0.5, self.pub_callback) 
        self._counter = 0
        ```
    2. Add another member function in the class for the callback
        ```py
        def pub_callback(self):
          self.get_logger().info(f"Message #{self._counter}: My name is ___")
          self._counter += 1
        ```
2. Initialize the publisher by adding this line to the constructor (above the two lines you just added)
    ```py
    self._publisher = self.create_publisher(String, "demo_topic", 10)
    ```
   **String:** We must specify the type of message we are going to send, like a data type. In this example we are publishing a `String`. You can create custom message types as well
   
   **demo_topic:** The name of the topic to publish messages to 

   **10:** 10 specifies the queue size. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough
3. We used the `String` message type, so we have to import the module 
    ```py
    from std_msgs.msg import String
    ```
4. Publish a message by adding this to the `pub_callback` to publish a message every 0.5 seconds
   ```py
    msg = String()
    name = "tiny hawk" # Change this to your name
    msg.data = f"{name}: {self._counter}"
    self._publisher.publish(msg)
   ```
5. Finally add the dependencies to `package.xml`
    ```xml
    <depend>rclpy</depend>  <!-- This should already be there -->
    <depend>std_msgs</depend>
    ```

#### Subscriber:
1. Initialize the publisher by adding this line to the constructor
    ```py
    self.subscription = self.create_subscription(String,"demo_topic", self.sub_callback, 10)
    ```
    Note that the topic name used here MUST match that of the publisher for them to communicate
2. Add another member function in the class for the callback. You do not need to start a timer because callback gets called as soon as it receives a message
    ```py 
    def sub_callback(self, msg):
          self.get_logger().info(f"Hello {msg.data}")
    ```
3. Be sure to import String in `sub_demo.py` as well
    ```py
    from std_msgs.msg import String
    ```

#### Run the program
1. Return to the workspace in your terminal and run the following to avoid having to re-build every time you wish to run the nodes 
    ```sh
    colcon build --symlink-install
    source ~/.bashrc
    ```
2. In one terminal run 
    ```sh 
    ros2 run pub_sub pub_demo
    ```
3. In another terminal run 
    ```
    ros2 run pub_sub sub_demo
    ```

#### Further inspection 
1. Open a new terminal and list the topics again, you should see our demo topic 
    ```sh
    ros2 topic list 
    ```
2. You can also view the graph with `rqt_graph` and open graphical tools
    ```py 
    rqt_graph
    ```
