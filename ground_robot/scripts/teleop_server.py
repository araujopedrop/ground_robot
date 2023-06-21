#!/usr/bin/python3

import rclpy

from rclpy.node              import Node
from std_msgs.msg            import Float64, String
from geometry_msgs.msg       import Twist
from my_robot_interfaces.srv import CmdVehicle



class teleopKeyServerNode(Node):

    VELOCITY = 0.0

    MAPPING = "Map"
    SAVE_MAP = "Save Map"
    NAVIGATE = "Nav"

    STRING_SLOW_SPEED = "SLOW_SPEED"
    STRING_NORMAL_SPEED = "NORMAL_SPEED"
    STRING_HIGH_SPEED = "HIGH_SPEED"

    SLOW_SPEED = 0.25
    NORMAL_SPEED = 1.0
    HIGH_SPEED = 2.0

    ACTION = 1

    def __init__(self):
        super().__init__("teleop_key_server_node")

        self.VELOCITY = self.NORMAL_SPEED

        self.pub_vel_     = self.create_publisher(Twist,"/cmd_vel",10)
        self.pub_cam_rot_ = self.create_publisher(Float64,"/cmd_camera",10)
        self.pub_act_vel_ = self.create_publisher(String,"/act_vel",10)

        self.cmd_service_cdm_movement = self.create_service(CmdVehicle,"cmd_vehicle_movement",self.check_cmd)
        self.cmd_service_cmd_speed    = self.create_service(CmdVehicle,"cmd_vehicle_velocity",self.check_cmd_velocity)

        self.timer_act_vel_ = self.create_timer(1.0,self.publish_act_vel)

        self.get_logger().info("teleop_key_service is up!")


    def publish_act_vel(self):
        msg = String()

        msg.data = "Actual velocity is set with value: " + str(self.VELOCITY)

        self.pub_act_vel_.publish(msg)

    def check_cmd(self,request,response):
        '''
            string command
            ---
            bool result
        '''

        key = request.command

        try:

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


        except Exception as e:
            self.get_logger().error("Could not execute command!")
            response.result = False

            return response

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
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.VELOCITY
        self.pub_vel_.publish(msg)

    def rotate_anti_clockwise(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -self.VELOCITY
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



    def check_cmd_velocity(self,request,response):
        '''
            string command
            ---
            bool result
        '''

        key = request.command

        try:

            if key == self.STRING_SLOW_SPEED:
                self.slow_down()
                response.result = True
            elif key == self.STRING_NORMAL_SPEED:
                self.normilize_speed()
                response.result = True
            elif key == self.STRING_HIGH_SPEED:
                self.speed_up()
                response.result = True
            else:
                response.result = False

            return response


        except Exception as e:
            self.get_logger().error("Could not execute command!")
            response.result = False

            return response

    def slow_down(self):
        self.VELOCITY = self.SLOW_SPEED

    def normilize_speed(self):
        self.VELOCITY = self.NORMAL_SPEED

    def speed_up(self):
        self.VELOCITY = self.HIGH_SPEED

def main(args=None):
    rclpy.init(args=args)
    node = teleopKeyServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
