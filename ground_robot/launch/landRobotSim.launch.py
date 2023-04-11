from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess

# Retrieving path information 
from ament_index_python.packages import get_package_share_directory
from pathlib import Path 

# Utilizing launch files from other packages
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Working with environment variables
from launch.actions import SetEnvironmentVariable

# Simulation event handling 
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

package_name = "ground_robot"

# Path Variables 
ignition_ros_package_path  = get_package_share_directory("ros_gz_sim")
udemy_ros2_pkg_path        = get_package_share_directory(package_name)
simulation_world_file_path = Path(udemy_ros2_pkg_path, "worlds/world_land_robot.sdf").as_posix()
simulation_models_path     = Path(udemy_ros2_pkg_path, "models").as_posix()

simulation = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
)


def generate_launch_description():
    ld = LaunchDescription()


    teleop_key_client_node = Node(
        package=package_name,
        executable="teleop_client.py",
        name="teleop_key"
    )

    teleop_key_server_node = Node(
        package=package_name,
        executable="teleop_server.py",
        name="teleop_key"
    )



    number_publisher_node = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                   "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                   "cmd_camera@std_msgs/msg/Float64@ignition.msgs.Double"], 
        remappings=[("/camera","/camera/image_raw")],
        output="screen"
    )

    set_env = SetEnvironmentVariable(
    name="IGN_GAZEBO_RESOURCE_PATH",
    value=simulation_models_path
    )

    simulation = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
    )
        

    #Para parar el roslaunch al cerrar la simulacion
    number_counter_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    ld.add_action(set_env)
    ld.add_action(simulation)
    ld.add_action(teleop_key_client_node)
    ld.add_action(teleop_key_server_node)

    return ld
