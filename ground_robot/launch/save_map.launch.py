from launch_ros.actions import Node 
from launch import LaunchDescription


def generate_launch_description():

    package_name = "nav2_map_server"

    ld = LaunchDescription()

    save_map_node = Node(
        package=package_name,
        executable="map_saver_cli",
        name="map_saver_cli",
        arguments=["-f","/home/ppa/ros2_ws_2/src/ground_robot/maps/my_map"]
    )

    ld.add_action(save_map_node)

    return ld


