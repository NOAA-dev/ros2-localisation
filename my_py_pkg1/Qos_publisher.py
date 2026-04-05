#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
 
 
class QOStesterNode(Node):
    def __init__(self):
        super().__init__("QOS_publisher") 

        self.qos_pub = QoSProfile(depth= 10)

        self.declare_parameter("Reliability","system_default")
        self.declare_parameter("durability","system_default")

        self.reliabilty_ = self.get_parameter("Reliability").value
        self.durability_ = self.get_parameter("durability").value

        if self.reliabilty_ == "BEST_EFFORT":
            self.qos_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
        elif self.reliabilty_ == "RELIABLE":
            self.qos_pub.reliability = QoSReliabilityPolicy.RELIABLE
        elif self.reliabilty_ == "system_default":
            self.qos_pub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        else:
            self.get_logger().error("their is no such reliabilty configuration")
            return
        
        if self.durability_ == "VOLATILE":
            self.qos_pub.durability = QoSDurabilityPolicy.VOLATILE
        elif self.durability_ == "TRANSIENT_LOCAL":
            self.qos_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        elif self.durability_ == "system_default":
            self.qos_pub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
        else:
            self.get_logger().error("their is no such durability configuration")
            return
        
        self.publisher_ = self.create_publisher(String, "QOS_test_topic", self.qos_pub)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.count_ = 0

    def publish_message(self):
        msg = String()
        msg.data = f"Hello, this is message number {self.count_}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.count_ += 1

 
def main(args=None):
    rclpy.init(args=args)
    node = QOStesterNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
