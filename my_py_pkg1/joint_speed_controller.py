#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped ,TransformStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler 
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

 
 
class jointspeedcontrollerNode(Node): 
    def __init__(self):
        super().__init__("joint_speed_controller")
        self.get_logger().info("joint_speed_controller node has been started")

        self.declare_parameter("wheel_radius",0.1)
        self.declare_parameter("wheel_seperation",0.42)
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_seperation = self.get_parameter("wheel_seperation").value

        self.multiplyer = 1.0
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time = None
        self.x_coodinate = 0.0
        self.y_coodinate = 0.0
        self.theta = 0.0
        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                           [self.wheel_radius/self.wheel_seperation, -self.wheel_radius/self.wheel_seperation]])
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

        self.subscription_ = self.create_subscription(TwistStamped,"/bumper_bot/cmd_vel",self.callback,10)
        self.subscriber_1 = self.create_subscription(Bool,"/simple_velocity_controller/gear_1",self.gear_1,10)
        self.subscriber_2 = self.create_subscription(Bool,"/simple_velocity_controller/gear_2",self.gear_2,10)
        self.subscriber_3 = self.create_subscription(JointState,"joint_states",self.position_callback,10)
        self.publisher_ = self.create_publisher(Float64MultiArray,"/simple_velocity_controller/commands",10)
        self.odom_publisher_ = self.create_publisher(Odometry,"bumper_bot/odom",10)


    def callback(self,msg: TwistStamped):
        V = msg.twist.linear.x
        W = msg.twist.angular.z

        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0]*self.multiplyer, wheel_speed[0, 0]*self.multiplyer]

        self.publisher_.publish(wheel_speed_msg)

    def gear_1(self,msg = Bool):
        if msg.data == True:
            self.get_logger().info("Gear 1 is selected")
            self.multiplyer = 1

    def gear_2(self,msg = Bool):
        if msg.data == True:
            self.get_logger().info("Gear 2 is selected")
            self.multiplyer = 2

    def position_callback(self,msg: JointState):

        try:
            idx_l = msg.name.index("base_left_wheel_joint")
            idx_r = msg.name.index("base_right_wheel_joint")
        except ValueError:
            return

        left_wheel_position = msg.position[idx_l]
        right_wheel_position = msg.position[idx_r]


        delta_R = right_wheel_position - self.right_wheel_prev_pos_
        delta_L = left_wheel_position - self.left_wheel_prev_pos_

        current_time = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        if dt <= 0.0:
            return



        self.left_wheel_prev_pos_ = left_wheel_position
        self.right_wheel_prev_pos_ = right_wheel_position



        delta_s = (self.wheel_radius * delta_R + self.wheel_radius*delta_L) / 2
        delta_theta = self.wheel_radius * (delta_R - delta_L) / self.wheel_seperation
        
        self.theta += delta_theta
        self.x_coodinate += delta_s * math.cos(self.theta)
        self.y_coodinate += delta_s * math.sin(self.theta)

        d_phi_R = delta_R / dt
        d_phi_L = delta_L / dt

        linear_velocity = (self.wheel_radius * d_phi_R + self.wheel_radius * d_phi_L) / 2
        angular_velocity =(self.wheel_radius * d_phi_R - self.wheel_radius * d_phi_L) / self.wheel_seperation

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x_coodinate
        self.odom_msg.pose.pose.position.y = self.y_coodinate
        self.odom_msg.twist.twist.linear.x = linear_velocity
        self.odom_msg.twist.twist.angular.z = angular_velocity

        r = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = r[0]
        self.odom_msg.pose.pose.orientation.y = r[1]
        self.odom_msg.pose.pose.orientation.z = r[2]
        self.odom_msg.pose.pose.orientation.w = r[3]

        self.odom_publisher_.publish(self.odom_msg)

        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped.transform.translation.x = self.x_coodinate
        self.transform_stamped.transform.translation.y = self.y_coodinate
        self.transform_stamped.transform.translation.z = 0.0
        self.transform_stamped.transform.rotation.x = r[0]
        self.transform_stamped.transform.rotation.y = r[1]
        self.transform_stamped.transform.rotation.z = r[2]
        self.transform_stamped.transform.rotation.w = r[3]
        self.tf_broadcaster.sendTransform(self.transform_stamped)


 
def main(args=None):
    rclpy.init(args=args)
    node = jointspeedcontrollerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
