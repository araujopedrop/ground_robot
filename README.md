# ground_robot
Simulation of a diff-drive robot with a camera sensor

There are two nodes:
- teleop_client: 
  - Displays a qt GUI for user commands
  - Sends commands to teleop_server using ROS services
- teleop_server
  - Receives commans from teleop_client
  - Publishes topics, such as /cmd_vel and /cmd_camera, to control the robot

Roslanches:
- landRobotSim.launch.py
