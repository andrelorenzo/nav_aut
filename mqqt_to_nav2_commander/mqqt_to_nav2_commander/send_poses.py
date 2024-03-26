
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Point, PointStamped
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseArray, PoseStamped,Pose
from robot_localization.srv import FromLL
from std_msgs.msg import String
from rclpy.duration import Duration
import time
import math

class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.mqtt_single_point_sub = self.create_subscription(
            PoseStamped, "/mqtt/goal", self.gps_point_callback, 1)
        self.mqtt_array_pose_sub = self.create_subscription(
            PoseArray, "/mqtt/goal_array", self.gps_point_array_callback, 1)

        self.get_logger().info('Ready for waypoints...')

    def gps_point_callback(self, msg: PoseStamped):
        self.navigator.goToPose(msg)
        
    def gps_point_array_callback(self, msg: PoseArray):
        array = []
        for p in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = p
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.navigator.get_clock().now().to_msg()
            array.append(pose_stamped)
            
        self.navigator.followWaypoints(array)
            
        
    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            

def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    gps_wpf.spin()


if __name__ == "__main__":
    main()
