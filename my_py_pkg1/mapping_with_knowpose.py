#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer, LookupException
from tf_transformations import euler_from_quaternion

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class Cordinate:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

def Cordinate_to_pose(translation, map_info: MapMetaData):
    pose = Cordinate(0,0)
    pose.x = round((translation.x - map_info.origin.position.x)/map_info.resolution)
    pose.y = round((translation.y - map_info.origin.position.y)/map_info.resolution)

    return pose

def onMap(pose: Cordinate, map_info: MapMetaData):
    if pose.x < 0 or pose.x >= map_info.width:
        return False
    if pose.y < 0 or pose.y >= map_info.height:
        return False
    return True

def to_cell_in_map_vector(pose: Cordinate, map_info: MapMetaData):
    cell_index = int(pose.y * map_info.width + pose.x)
    return cell_index

def bresenham(start: Cordinate, end: Cordinate):
    line = []
    dx = end.x - start.x
    dy = end.y - start.y
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1
    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Cordinate(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx

        D += 2 * dy

    return line

def mark_cells(robot_pose: Cordinate, beam_pose: Cordinate):
    line = bresenham(robot_pose, beam_pose)
    occupied = []
    for pose in line[:-1]:
        occupied.append((pose,FREE_PROB))
    occupied.append((line[-1],OCC_PROB))
    return occupied


    
 
class MappingwithKnowPoseNode(Node): 
    def __init__(self):
        super().__init__("mapping_with_knowpose") 

        self.qos_sub = QoSProfile(depth= 10)

        self.declare_parameter("Reliability","BEST_EFFORT")
        self.declare_parameter("durability","VOLATILE")

        self.reliabilty_ = self.get_parameter("Reliability").value
        self.durability_ = self.get_parameter("durability").value
        self.qos_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT if self.reliabilty_ == "BEST_EFFORT" else QoSReliabilityPolicy.RELIABLE
        self.qos_sub.durability = QoSDurabilityPolicy.VOLATILE if self.durability_ == "VOLATILE" else QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.declare_parameter("length",50.0)
        self.declare_parameter("width",50.0)
        self.declare_parameter("resolution",0.1)

        self.length_ = self.get_parameter("length").value
        self.width_ = self.get_parameter("width").value
        self.resolution_ = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.header.frame_id = "odom"
        self.map_.info.resolution = self.resolution_
        self.map_.info.height = round(self.length_/self.resolution_)
        self.map_.info.width = round(self.width_/self.resolution_)
        self.map_.info.origin.position.x = float(-round(self.width_/2.0))
        self.map_.info.origin.position.y = float(-round(self.length_/2.0))
        self.map_.data = [-1]*(self.map_.info.width*self.map_.info.height)
        self.probability_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.map_pub_ = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub_ = self.create_subscription(LaserScan, "scan", self.scan_callback, self.qos_sub)
        self.timer_ = self.create_timer(0.1, self.publish_map)


    def scan_callback(self, msg: LaserScan):
        try:
            tf_ = self.tf_buffer_.lookup_transform(self.map_.header.frame_id, msg.header.frame_id, rclpy.time.Time())
        except LookupException as error:
            self.get_logger().error(f"Transform lookup failed: {error}")
            return
        
        robot_pose = Cordinate_to_pose(tf_.transform.translation,self.map_.info)

        if not onMap(robot_pose, self.map_.info):
            self.get_logger().warn("the robot is not on the map !ping! ")
            return
        
        cell_ = to_cell_in_map_vector(robot_pose, self.map_.info)
        self.map_.data[cell_] = 0

        (r,p,y) = euler_from_quaternion([tf_.transform.rotation.x, tf_.transform.rotation.y, tf_.transform.rotation.z, tf_.transform.rotation.w])
        object_pose = Pose()

        for i in range(len(msg.ranges)):
            if math.isinf(msg.ranges[i]) or math.isnan(msg.ranges[i]) or msg.ranges[i] > msg.range_max or msg.ranges[i] < msg.range_min:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment + y
            object_pose.position.x = tf_.transform.translation.x + (msg.ranges[i] * math.cos(angle))
            object_pose.position.y = tf_.transform.translation.y + (msg.ranges[i] * math.sin(angle))

            object_ = Cordinate_to_pose(object_pose.position,self.map_.info)

            if not onMap(object_, self.map_.info):
                continue
            
            cell_info = mark_cells(robot_pose,object_)
            for pose , value in cell_info:
                cell_ = to_cell_in_map_vector(pose, self.map_.info)
                self.probability_map_[cell_] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

    def publish_map(self):
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probability_map_]
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub_.publish(self.map_)
 
def main(args=None):
    rclpy.init(args=args)
    node = MappingwithKnowPoseNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
