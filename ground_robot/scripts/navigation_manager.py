#!/usr/bin/python3



import rclpy
import time
import launch

import threading
import subprocess

from rclpy.node                        import Node
from signal                            import SIGINT
from std_msgs.msg                      import String
from my_robot_interfaces.srv           import CmdVehicle

from launch                            import LaunchService
from launch.actions                    import ExecuteProcess
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

    ls_mapping    = LaunchService()
    ls_saving_map = LaunchService()
    ls_navigate   = LaunchService()

    def __init__(self):
        super().__init__("navigation_manager_node")

        self.cmd_service_ = self.create_service(CmdVehicle,"nav_cmd",self.check_cmd)
        self.pub_cmd_ = self.create_publisher(String,"last_cmd",10)
        #self.timer_pub_ = self.create_timer(0.1,self.publish_last_cmd)

        self.LAST_CMD = "Modo normal"

        self.get_logger().info("nav_cmd service is up!")

    def publish_last_cmd(self):
        msg = String()
        msg.data = "navigation_manager_node: " + self.LAST_CMD

        self.pub_cmd_.publish(msg)

    def check_cmd(self,request,response):

        cmd = request.command
        self.LAST_CMD = "Me llego algo"

        try:

            if cmd == self.MAPPING:
                response.result = self.launch_mapping()
                return response

            elif cmd == self.SAVE_MAP:
                response.result = self.launch_saving_map()
                return response

            elif cmd == self.NAVIGATE:
                response.result = self.launch_saving_map()
                return response

            else:
                self.LAST_CMD = "No se que me llego"
                self.get_logger().warn("check_cmd: Could not execute command! Command not available = " + str(cmd))
                response.result = "No se que me llego"

                return response

        except Exception as e:
            self.get_logger().error("check_cmd: Could not execute command! Error")
            response.result = "Error"

            self.LAST_CMD = "Error"

            return response
        
    def launch_mapping(self):

        try: 

            # Op 1
            '''
            package_name = "ground_robot"

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(package_name), '/launch', '/mapping.launch.py'])
                        #FindPackageShare(package_name), '/launch', '/test.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls_mapping.include_launch_description(ld)

            # 
            #self.ls_mapping.run()

            '''

            # Op 2
            self.launch_process = subprocess.Popen(["ros2", "launch", "ground_robot", "mapping.launch.py"], text=True)


            # Op 3
            '''
            ld = LaunchDescription()


            simulation = ExecuteProcess(
            cmd=['ros2', 'launch', 'ground_robot','mapping.launch.py']
            )

            ld.add_action(simulation)
            self.ls_mapping.include_launch_description(ld)
            self.ls_mapping.run()
            '''

            # Op 4
            # self.ls_mapping.run_async() ???


            self.LAST_CMD = "Mapeo"

            return "Mapeo"

        except Exception:

            self.LAST_CMD = "Error realizando Mapeo"

            return "Error Mapeo"

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

            self.ls_saving_map.include_launch_description(ld)

            self.ls_saving_map.run()

            self.LAST_CMD = "Mapa guardado"

            time.sleep(3)


            # -------------------- Closing mapping launch file --------------------

            try:

                #self.ls_mapping.shutdown()
                self.ls_saving_map.shutdown()
                
                #self.launch_process.send_signal(SIGINT)
                self.launch_process.terminate()
                self.launch_process.wait(timeout=30)
                if self.launch_process.poll() is None:
                    self.launch_process.kill()
                self.get_logger().warn("Launchfile was shutdowned")
                self.LAST_CMD = "Guardar mapa"
                

                return "Guardando Mapa"

            except Exception:
                self.LAST_CMD = "Error realizando Guardado de mapa"

                return "Error Guardando Mapa"
            
        except Exception:

            self.LAST_CMD = "Error realizando Guardado de mapa"

            return "Error Guardando Mapa"

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

            return "Navigating"

        except Exception:

            self.LAST_CMD = "Error realizando Guardado de mapa"

            return "Error in Navigating"


def main(args=None):
    rclpy.init(args=args)
    node = navigationManagerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
