#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci
 
 
class MyActionServerNode(Node): 
    def __init__(self):
        super().__init__("my_action_server") 

        self.action_server_ = ActionServer(self, Fibonacci, "fibonacci", self.execute_callback)
        self.get_logger().info("Action Server has been started! PING!! ")

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0,1]
        
        for i in range(1,goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i]+feedback_msg.partial_sequence[i-1])
            self.get_logger().info("feedback {0}".format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result_ = Fibonacci.Result()
        result_.sequence = feedback_msg.partial_sequence

        return result_
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MyActionServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
