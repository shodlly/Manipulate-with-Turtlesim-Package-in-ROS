
# Manipulate with Turtlesim Package in ROS1 Noetic

## Table of Contents

1. [Introduction](#introduction)
2. [Installation and Setup](#installation-and-setup)
3. [Work Overview](#work-overview)
   - [Starting Turtlesim](#starting-turtlesim)
   - [Manual Control](#manual-control)
   - [Moving the Turtle](#moving-the-turtle)
   - [Teleporting the Turtle](#teleporting-the-turtle)
   - [Changing Background Color](#changing-background-color)
   - [Resetting the Simulation](#resetting-the-simulation)
   - [Drawing Shapes](#drawing-shapes)
4. [Core Concepts](#core-concepts)
   - [ROS Nodes](#ros-nodes)
   - [ROS Topics](#ros-topics)
   - [ROS Services](#ros-services)
   - [ROS Parameters](#ros-parameters)
   - [ROS Tools](#ros-tools)
5. [Conclusion](#conclusion)

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

To customize the appearance of the simulation environment, I changed the background color using ROS parameters.

- **Set the Background Color:**

  ```sh
  rosparam set /background_r 255
  rosparam set /background_g 255
  rosparam set /background_b 255
  rosservice call /clear
  ```

  These commands set the RGB values for the background color to white (255, 255, 255) and then call the `/clear` service to apply the changes. This demonstrated the use of parameters to modify environment settings dynamically.

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

## Core Concepts

### ROS Nodes

Nodes are the fundamental building blocks of a ROS system, representing processes that perform computations. In this project:

- **List All Active Nodes:**

  ```sh
  rosnode list
  ```

  This command lists all active nodes in the system. It was used to confirm that nodes like `/turtlesim` and `/turtle_teleop` were running.

- **Get Information About a Specific Node (`/turtlesim`):**

  ```sh
  rosnode info /turtlesim
  ```

  This command provides detailed information about the `/turtlesim` node, such as its publications, subscriptions, and connections. This was useful for understanding the interactions and dependencies of the node.

### ROS Topics

Topics in ROS are used for communication between nodes through message passing.

- **List All Active Topics:**

  ```sh
  rostopic list
  ```

  This command lists all topics currently being published. It was used to identify relevant topics like `/turtle1/cmd_vel` and `/turtle1/pose`.

- **Display Messages Published on a Topic (`/turtle1/pose`):**

  ```sh
  rostopic echo /turtle1/pose
  ```

  This command displays real-time data being published on the `/turtle1/pose` topic, which provides the turtle's current position and orientation. It helped monitor the turtle's state during various manipulations.

- **Publish a Message to a Topic (`/turtle1/cmd_vel`):**

  ```sh
  rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
  ```

  This command sends a `Twist` message to the `/turtle1/cmd_vel` topic, directing the turtle's motion. It was essential for controlling the turtle programmatically.

### ROS Services

Services in ROS allow nodes to send a request and receive a response, enabling more direct control.

- **List All Available Services:**

  ```sh
  rosservice list
  ```

  This command lists all active services. It was used to identify services like `/turtle1/teleport_absolute` and `/reset`.

- **Get Information About a Specific Service (`/turtle1/teleport_absolute`):**

  ```sh
  rosservice info /turtle1/teleport_absolute
  ```

  This command provides details about the `/turtle1/teleport_absolute` service, such as its service type and nodes that provide and use it. It was useful for understanding how to use this service for teleportation.

- **Call a Service (`/turtle1/teleport_absolute`):**

  ```sh
  rosservice call /turtle1/teleport_absolute 5.5 5.5 0.0
  ```

  This command calls the `/turtle1/teleport_absolute` service to set
