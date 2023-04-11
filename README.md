# ground_robot
Simulation of a diff-drive robot with a camera sensor

Roslanches:
- landRobotSim.launch.py
  - Lanches node teleop_client
  - node teleop_server
  - node parameter_bridge
  - Gazebo
  - Sets environment variables
  - Uses event handlers
  
Nodes:
- teleop_client: 
  - Displays a qt GUI for user commands
  - Sends commands to teleop_server using ROS services
- teleop_server:
  - Receives commans from teleop_client
  - Publishes topics, such as /cmd_vel and /cmd_camera, to control the robot
- parameter_bridge:
  - From ros_ign_bridge package
  - Connects gazebo topics with ros topics 

Services:
- CmdVehicle.srv

Worlds:
- Includes:
  - ground_robot (land_robot)
  - A sun
  - A ground

