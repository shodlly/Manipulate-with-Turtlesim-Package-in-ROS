

# Manipulate with Turtlesim Package in ROS1 Noetic

## Table of Contents

Introduction
Installation and Setup
Work Overview
Starting Turtlesim
Manual Control
Moving the Turtle
Teleporting the Turtle
Changing Background Color
Resetting the Simulation
Drawing Shapes
Python Nodes: turtle_controller.py and turtle_state_publisher.py
Spawning Turtles and Simulating Multiple Heads
Visualizing ROS Communication
Core Concepts
ROS Nodes
ROS Topics
ROS Services
ROS Parameters
ROS Tools
Conclusion

## Introduction

This README details the specific tasks and commands executed during the manipulation of the `turtlesim` package in ROS. The focus is on the practical aspects and applications I explored, demonstrating my understanding and use of ROS concepts.

## Installation and Setup

Ensure ROS Noetic is installed and the necessary packages are set up.

```sh
sudo apt update
sudo apt install ros-noetic-turtlesim
source /opt/ros/noetic/setup.bash
```

To automatically source ROS setup files:

```sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Work Overview

### Starting Turtlesim

To begin my work with `turtlesim`, I started the simulation environment by launching the main node responsible for the turtlesim window.

- **Launch Turtlesim Node (`/turtlesim`):**

  ```sh
  rosrun turtlesim turtlesim_node
  ```

  This command starts the `turtlesim_node`, which creates a window displaying a turtle in a simulated environment. This node is essential as it provides the graphical interface for interacting with the turtle.

### Manual Control

To manually control the turtle, I used a keyboard teleoperation node. This allows for direct interaction with the turtle's movement.

- **Manual Control using Keyboard (`/turtle_teleop`):**

  ```sh
  rosrun turtlesim turtle_teleop_key
  ```
![image](https://github.com/user-attachments/assets/9ad79806-d6cf-427f-b53f-b3943a7ea134)

  This command launches the `turtle_teleop_key` node, which enables control of the turtle using keyboard arrow keys. This was used to manually maneuver the turtle around the screen, helping to understand real-time movement dynamics.

### Moving the Turtle

To control the turtle's movement programmatically, I published messages to the `/turtle1/cmd_vel` topic, which controls the turtle's linear and angular velocity.

- **Move the Turtle with Linear and Angular Velocity:**

  ```sh
  rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
  ```

  This command publishes a `Twist` message to the `/turtle1/cmd_vel` topic, causing the turtle to move forward with a linear velocity of 2.0 units and rotate with an angular velocity of 1.8 radians per second. This method was used to programmatically direct the turtle's motion without manual intervention.

### Teleporting the Turtle

I used ROS services to teleport the turtle to specific coordinates instantly, bypassing physical movement.

- **Teleport to a Specific Position:**

  ```sh
  rosservice call /turtle1/teleport_absolute 5.5 5.5 0.0
  ```

  This command calls the `/turtle1/teleport_absolute` service, setting the turtle's position to coordinates (5.5, 5.5) and orientation to 0 radians. It was useful for quickly repositioning the turtle within the environment.

- **Teleport Relative to the Current Position:**

  ```sh
  rosservice call /turtle1/teleport_relative 2.0 1.57
  ```

  This command teleports the turtle 2.0 units forward and rotates it by 1.57 radians (90 degrees). It helped in scenarios requiring incremental movements.

### Changing Background Color

To customize the appearance of the simulation environment, I changed the background color to Orange using ROS parameters.

- **Set the Background Color:**

  ```sh
  rosparam set /background_r 255
  rosparam set /background_g 165
  rosparam set /background_b 0
  rosservice call /clear
  ```
![image](https://github.com/user-attachments/assets/5838d0a5-4385-4f69-a959-fb7a5f53cdef)

  These commands set the RGB values for the background color to Orange (255, 165, 0) and then call the `/clear` service to apply the changes. This demonstrated the use of parameters to modify environment settings dynamically.

### Resetting the Simulation

To reset the simulation environment to its initial state:

- **Reset Turtlesim:**

  ```sh
  rosservice call /reset
  ```

  This command calls the `/reset` service, which resets the turtle's position and the background to their original states. This was useful for starting fresh after testing various commands.

### Drawing Shapes

I programmed the turtle to draw shapes, like circles and squares, by controlling its movement.

- **Draw a Circle:**

  ```sh
  rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 2.0}, angular: {z: 2.0}}'
  ```
![image](https://github.com/user-attachments/assets/5b84514b-4ee9-4215-aa69-0f49f600ed2a)

  This command continuously publishes messages to make the turtle move in a circular path, illustrating the use of constant linear and angular velocities.

- **Draw a Square (Python Script):**

  ```python
  import rospy
  from geometry_msgs.msg import Twist

  rospy.init_node('draw_square')
  pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
  rate = rospy.Rate(1)
  
  move_cmd = Twist()
  move_cmd.linear.x = 2.0

  turn_cmd = Twist()
  turn_cmd.angular.z = 1.57  # 90 degrees in radians

  for _ in range(4):
      pub.publish(move_cmd)
      rospy.sleep(2)
      pub.publish(turn_cmd)
      rospy.sleep(1)
  ```

  This Python script was used to draw a square by alternating between moving straight and turning 90 degrees.

### Python Nodes: `turtle_controller.py` and `turtle_state_publisher.py`

In addition to the manual and programmatic control methods, I developed two Python nodes to control the turtle and publish its state.

#### `turtle_controller.py`

- **Purpose:** Controls the turtle's movement by publishing velocity commands to the `/turtle1/cmd_vel` topic.

- **Code:**

  ```python
  #!/usr/bin/env python

  import rospy
  from geometry_msgs.msg import Twist

  def move_turtle():
      rospy.init_node('turtle_controller', anonymous=True)
      pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
      rate = rospy.Rate(1)

      while not rospy.is_shutdown():
          vel_msg = Twist()
          vel_msg.linear.x = 2.0
          vel_msg.angular.z = 1.0
          
          pub.publish(vel_msg)
          rate.sleep()

  if __name__ == '__main__':
      try:
          move_turtle()
      except rospy.ROSInterruptException:
          pass
  ```

  - **Role:** This node publishes commands to move the turtle forward and rotate it. It demonstrates programmatic control and continuous publishing of velocity commands.

#### `turtle_state_publisher.py`

- **Purpose:** Monitors the turtle’s pose and publishes its state as a string message.

- **Code:**

  ```python
  #!/usr/bin/env python

  import rospy
  from turtlesim.msg import Pose
  from std_msgs.msg import String

  def pose_callback(data):
      state_msg = "Position: (%.2f, %.2f), Orientation: %.2f" % (data.x, data.y, data.theta)
      rospy.loginfo(state_msg)
      state_pub.publish(state_msg)

  def state_publisher():
      global state_pub
      rospy.init_node('turtle_state_publisher', anonymous=True)
      rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
      state_pub = rospy.Publisher('/turtle1/state', String, queue_size=10)
      rospy.spin()

  if __name__ == '__main__':
      try:
          state_publisher()
      except rospy.ROSInterruptException:
          pass
  ```

- **Role:** This node subscribes to the `/turtle1/pose` topic, which provides the turtle's position and orientation data. It publishes this data as a string message to the `/turtle1/state` topic, allowing other nodes to access the turtle's state information.

## Visualizing ROS Communication

To generate a graph of ROS communication using `rqt_graph`, follow these steps:

1. **Start the ROS Master and Nodes:**
   Make sure that the ROS Master is running and that you have launched all the necessary nodes (e.g., `turtlesim_node`, `turtle_teleop_key`, `turtle_controller.py`, etc.).

   ```sh
   roscore
   ```
   In a new terminal, start your nodes:
   ```sh
   rosrun turtlesim turtlesim_node
   rosrun your_package turtle_controller.py
   rosrun your_package turtle_state_publisher.py
   ```

2. **Open `rqt_graph`:**
   Open another terminal and run the following command to start `rqt_graph`:

   ```sh
   rosrun rqt_graph rqt_graph
   ```

   This command will open a graphical interface that displays the ROS nodes and the topics they communicate over.

3. **View the Graph:**
   In the `rqt_graph` window, you will see a visual representation of nodes and topics. You can interact with this graph to understand how the nodes are connected and which topics are being published or subscribed to.

The graph below illustrates the communication between the nodes and topics involved in the turtle's movement:

![image](https://github.com/user-attachments/assets/3e042d78-25c9-4e60-94ae-85b14738848a)

### Graph Output Explanation

**First Circle: Turtle Controller**
- This node represents the control system used to publish velocity commands to the turtle.

**Second Circle: Turtlesim**
- This node receives the commands and updates the turtle's state accordingly.

The line connecting these circles represents the `/turtle1/cmd_vel` topic through which the velocity commands are published and received. This communication allows the turtle to move based on the instructions sent by the controller.

### Role of Communication

The communication via the `/turtle1/cmd_vel` topic directly affects the turtle's movement. The Turtle Controller node publishes messages to this topic, specifying the desired linear and angular velocities. The Turtlesim node subscribes to this topic, interprets the messages, and adjusts the turtle's motion in the simulation environment. This seamless communication ensures that the turtle responds to commands in real-time, allowing for dynamic control and manipulation.

Here's a formatted README section for adding to your project that includes instructions on how to spawn two turtles and simulate one having three heads:

---

## Spawning Turtles and Simulating Multiple Heads

This section describes how to spawn two turtles in the `turtlesim` environment and customize one to simulate having three heads.

### Prerequisites

Ensure that your ROS workspace and package are set up. Follow the instructions below to create a Python script that handles spawning and modifying the turtles.

### Creating and Editing the Python Script

1. **Navigate to Your Package's Scripts Directory**

   ```sh
   cd ~/catkin_ws/src/my_turtle_sim/scripts
   ```

2. **Create the Python Script**

   ```sh
   touch spawn_and_modify_turtles.py
   chmod +x spawn_and_modify_turtles.py
   ```

3. **Edit the Python Script**

   Open `spawn_and_modify_turtles.py` with your preferred text editor (e.g., `nano`):

   ```sh
   nano spawn_and_modify_turtles.py
   ```

4. **Add the Following Code**

   ```python
   #!/usr/bin/env python

   import rospy
   from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
   from std_srvs.srv import Empty

   def spawn_turtles():
       rospy.init_node('spawn_and_modify_turtles_node')

       # Wait for the services to be available
       rospy.wait_for_service('/spawn')
       rospy.wait_for_service('/turtle1/set_pen')
       rospy.wait_for_service('/turtle1/teleport_absolute')
       rospy.wait_for_service('/turtle2/set_pen')
       rospy.wait_for_service('/turtle2/teleport_absolute')
       rospy.wait_for_service('/clear')

       try:
           # Create service proxies
           spawn = rospy.ServiceProxy('/spawn', Spawn)
           set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
           teleport_absolute = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
           set_pen_turtle2 = rospy.ServiceProxy('/turtle2/set_pen', SetPen)
           teleport_absolute_turtle2 = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
           clear = rospy.ServiceProxy('/clear', Empty)

           # Clear the screen
           clear()

           # Spawn two turtles
           spawn(2.0, 2.0, 0.0, 'turtle1')  # x, y, theta, name
           spawn(5.0, 5.0, 0.0, 'turtle2')

           # Modify turtle1 to simulate 3 heads
           set_pen(r=255, g=0, b=0, width=5, off=0)  # Red color, width 5

           for angle in range(0, 360, 120):  # Simulate heads in a circular pattern
               x = 2.0 + 1.0 * rospy.cos(angle * rospy.pi / 180)
               y = 2.0 + 1.0 * rospy.sin(angle * rospy.pi / 180)
               teleport_absolute(x, y, 0.0)
               rospy.sleep(0.5)

           # Modify turtle2 (just a simple move to show it's separate)
           teleport_absolute_turtle2(5.0, 5.0, 0.0)  # Move turtle2 to a new position
           set_pen_turtle2(r=0, g=255, b=0, width=3, off=0)  # Green color, width 3
           rospy.sleep(1)

       except rospy.ServiceException as e:
           print("Service call failed: %s" % e)

   if __name__ == "__main__":
       spawn_turtles()
   ```

   **Explanation:**
   - `/spawn`: Service used to spawn new turtles.
   - `/turtle1/set_pen` and `/turtle1/teleport_absolute`: For modifying the appearance and position of the first turtle.
   - `/turtle2/set_pen` and `/turtle2/teleport_absolute`: For modifying the appearance and position of the second turtle.
   - `/clear`: Clears the canvas.

### Running the Python Script

1. **Start ROS Core**

   ```sh
   roscore
   ```

2. **Launch Turtlesim**

   In a new terminal, run:

   ```sh
   rosrun turtlesim turtlesim_node
   ```

3. **Run the Python Script**

   In another terminal, execute:

   ```sh
   rosrun my_turtle_sim spawn_and_modify_turtles.py
   ```

### Observing the Results

![image](https://github.com/user-attachments/assets/26e556b4-4214-4fce-87a5-785b5b6b4067)
- **Turtle1**: Should appear to have three heads, simulated by drawing commands.
- **Turtle2**: A simple turtle with a different appearance.

---

## Core Concepts

### ROS Nodes

Nodes are processes that perform computation. In the turtlesim example, nodes are used to control the turtle and publish its state. 

### ROS Topics

Topics are named buses over which nodes exchange messages. For example, `/turtle1/cmd_vel` is a topic for sending velocity commands to the turtle, and `/turtle1/pose` is a topic for receiving the turtle’s position and orientation.

### ROS Services

Services are a way to provide synchronous remote procedure calls. For instance, the `/turtle1/teleport_absolute` and `/turtle1/reset` services allow for direct commands to be sent to the turtle for specific actions.

### ROS Parameters

Parameters are used to store configuration values that nodes can use. For example, background color settings for the simulation environment are set using parameters.

### ROS Tools

Various tools are used to interact with ROS, such as `rostopic`, `rosservice`, and `rosparam`, which help in inspecting topics, calling services, and setting parameters, respectively.

Conclusion
This guide provided a hands-on look at using the turtlesim package in ROS1 Noetic. it covered basic setup, manual and programmatic turtle control, and advanced features like spawning multiple turtles and customizing the environment. These exercises offer a solid foundation for working with ROS simulations and controlling robotic systems.
