#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,Twist
 
 
class CMDRelayNode(Node): 
    def __init__(self):
        super().__init__("cmd_relay") 

        self.get_logger().info("cmd_relay node has been started")

        self.joy_subscriber_ = self.create_subscription(Twist,"/bumper_bot/cmd_vel_unstamped",self.joy_callback,10)
        self.joy_in_subscriber_ = self.create_subscription(TwistStamped,"/joy_in_teleop/cmd_vel_stamped",self.joy_in_callback,10)

        self.cmd_vel_publisher_ = self.create_publisher(TwistStamped,"/bumper_bot/cmd_vel",10)
        self.joy_in_publisher_ = self.create_publisher(Twist,"/joy_in_teleop/cmd_vel_unstamped",10)
 
    def joy_callback(self,msg: Twist):
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.twist = msg
        self.cmd_vel_publisher_.publish(cmd_vel_msg)

    def joy_in_callback(self,msg: TwistStamped):
        joy_in_msg = Twist()
        joy_in_msg = msg.twist
        self.joy_in_publisher_.publish(joy_in_msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = CMDRelayNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
