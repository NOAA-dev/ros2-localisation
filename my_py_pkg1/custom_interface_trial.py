#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trial_interfaces.msg import HardwareStatus
 
class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("Status_publisher") 
        self.publisher_ = self.create_publisher(HardwareStatus,"Status",10)
        self.timer_ = self.create_timer(0.5,self.publish_news)
        self.get_logger().info("Status publisher station started. ")

    def publish_news(self):
        msg = HardwareStatus()
        msg.temperature = 70.0
        msg.is_on = True
        msg.debug_message = "mom has gastroestrophegalreflux disease!! "
        self.publisher_.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()