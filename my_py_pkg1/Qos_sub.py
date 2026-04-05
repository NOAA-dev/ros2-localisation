#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
 
 
class QOStesterNode(Node):
    def __init__(self):
        super().__init__("QOS_subscriber") 

        self.qos_sub = QoSProfile(depth= 10)

        self.declare_parameter("Reliability","system_default")
        self.declare_parameter("durability","system_default")

        self.reliabilty_ = self.get_parameter("Reliability").value
        self.durability_ = self.get_parameter("durability").value

        if self.reliabilty_ == "BEST_EFFORT":
            self.qos_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
        elif self.reliabilty_ == "RELIABLE":
            self.qos_sub.reliability = QoSReliabilityPolicy.RELIABLE
        elif self.reliabilty_ == "system_default":
            self.qos_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        else:
            self.get_logger().error("their is no such reliabilty configuration")
            return
        
        if self.durability_ == "VOLATILE":
            self.qos_sub.durability = QoSDurabilityPolicy.VOLATILE
        elif self.durability_ == "TRANSIENT_LOCAL":
            self.qos_sub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        elif self.durability_ == "system_default":
            self.qos_sub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
        else:
            self.get_logger().error("their is no such durability configuration")
            return
        
        self.subscription_ = self.create_subscription(String, "QOS_test_topic", self.subscribe_message, self.qos_sub)

    def subscribe_message(self, msg: String):
        self.get_logger().info("the recieved msg is: "+ (msg.data))

 
def main(args=None):
    rclpy.init(args=args)
    node = QOStesterNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
