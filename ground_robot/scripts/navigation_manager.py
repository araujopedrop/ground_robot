#!/usr/bin/python3



import rclpy
import time
import launch

import subprocess

import threading


from rclpy.node                        import Node
from signal                            import SIGINT
from std_msgs.msg                      import String
from my_robot_interfaces.srv           import CmdVehicle

from launch                            import LaunchService
from launch_ros.substitutions          import FindPackageShare
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

class navigationManagerNode(Node):

    MAPPING  = "Map"
    SAVE_MAP = "Save Map"
    NAVIGATE = "Nav"

    LAST_CMD = ""

    VELOCITY = 1.0
    ACTION = 1

    ls = LaunchService()

    def __init__(self):
        super().__init__("navigation_manager_node")

        self.cmd_service_ = self.create_service(CmdVehicle,"nav_cmd",self.check_cmd)
        self.pub_cmd_ = self.create_publisher(String,"last_cmd",10)
        self.timer_pub_ = self.create_timer(0.1,self.publish_last_cmd)

        self.LAST_CMD = "Modo normal"

        self.get_logger().info("nav_cmd service is up!")

    def publish_last_cmd(self):
        msg = String()
        msg.data = self.LAST_CMD

        self.pub_cmd_.publish(msg)

    def check_cmd(self,request,response):

        cmd = request.command
        self.LAST_CMD = "Me llego algo"

        try:

            if cmd == self.MAPPING:
                self.launch_mapping()

            elif cmd == self.SAVE_MAP:
                self.launch_saving_map()

            elif cmd == self.NAVIGATE:
                self.launch_saving_map()

            else:
                self.LAST_CMD = "No se que me llego"
                self.get_logger().warn("check_cmd: Could not execute command! Command not available = " + str(cmd))
                response.result = False

                return response

            response.result = True

            return response

        except Exception as e:
            self.get_logger().error("check_cmd: Could not execute command! Error")
            response.result = False

            self.LAST_CMD = "Error"

            return response
        

        
    def launch_mapping(self):

        try:

            package_name = "ground_robot"

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        #FindPackageShare(package_name), '/launch', '/mapping.launch.py'])
                        FindPackageShare(package_name), '/launch', '/test.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls.include_launch_description(ld)

            # 
            #self.ls.run()

            self.launch_process = subprocess.Popen(["ros2", "launch", "ground_robot", "mapping.launch.py"], text=True)

            self.LAST_CMD = "Mapeo"

        except Exception:

            self.LAST_CMD = "Error realizando Mapeo"

    def launch_saving_map(self):

        try:
        
            package_name = "ground_robot"

            # -------------------- Starting launch file for saving map --------------------

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(package_name), '/launch', '/save_map.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls.include_launch_description(ld)

            self.ls.run()

            self.LAST_CMD = "Mapa guardado"

            time.sleep(5)


            # -------------------- Closing mapping launch file --------------------

            #self.ls.shutdown()

            self.launch_process.send_signal(SIGINT)
            self.launch_process.wait(timeout=30)
            self.get_logger().warn("Launchfile was shutdowned")
            self.LAST_CMD = "Guardar mapa"
        
        except Exception:

            self.LAST_CMD = "Error realizando Guardado de mapa"

    def launch_navigate(self):

        try:
        
            package_name = "ground_robot"

            # -------------------- Starting launch file for saving map --------------------

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(package_name), '/launch', '/navigate.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls.include_launch_description(ld)

            self.ls.run()

            self.LAST_CMD = "Navegando"

        except Exception:

            self.LAST_CMD = "Error realizando Guardado de mapa"




def main(args=None):
    rclpy.init(args=args)
    node = navigationManagerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
