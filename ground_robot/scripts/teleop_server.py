#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from my_robot_interfaces.srv import CmdVehicle

class teleopKeyServerNode(Node):

    VELOCITY = 1.0
    ACTION = 1

    def __init__(self):
        super().__init__("teleop_key_server_node")

        self.pub_vel_ = self.create_publisher(Twist,"/cmd_vel",10)
        self.pub_cam_rot_ = self.create_publisher(Float64,"/cmd_camera",10)

        self.cmd_service_ = self.create_service(CmdVehicle,"cmd_vehicle",self.check_cmd)

        self.get_logger().info("teleop_key_service is up!")


    def check_cmd(self,request,response):
        '''
            string command
            ---
            bool result
        '''

        key = request.command

        #try:

        if key == 'w':
            self.go_forwards()
        elif key == 's':
            self.go_backwards()
        elif key == 'a':
            self.rotate_anti_clockwise()
        elif key == 'd':
            self.rotate_clockwise()
        elif key == 'space_bar':
            self.stop_vehicle()
        else:
            list = str(key).split()
            if list[0] == "->":
                cam_rads = float(list[1])
                self.cam_rotate(cam_rads)


        response.result = True

        return response

        '''
        except Exception as e:
            self.get_logger().error("Could not execute command!")
            response.result = False

            return response
        '''

    def go_forwards(self):
        msg = Twist()
        msg.linear.x = self.VELOCITY
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel_.publish(msg)

    def go_backwards(self):
        msg = Twist()
        msg.linear.x = -self.VELOCITY
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel_.publish(msg)


    def rotate_clockwise(self):
        msg = Twist()
        msg.linear.x = self.VELOCITY
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 5.0
        self.pub_vel_.publish(msg)

    def rotate_anti_clockwise(self):
        msg = Twist()
        msg.linear.x = self.VELOCITY
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -5.0
        self.pub_vel_.publish(msg)

    def stop_vehicle(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel_.publish(msg)

    def cam_rotate(self,cam_rads):

        msg = Float64()
        msg.data = cam_rads
        self.get_logger().info("str(cam_rads)")
        self.pub_cam_rot_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = teleopKeyServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
