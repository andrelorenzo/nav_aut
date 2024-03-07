
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Point, PointStamped
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseArray, PoseStamped,Pose
from robot_localization.srv import FromLL
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
            Point, "/mqtt/point", self.gps_point_callback, 1)
        
        self.mqtt_multiple_points_sub = self.create_subscription(
            PoseArray, "/mqtt/points", self.gps_points_callback, 1)
        
        self.localizer = self.create_client(FromLL,  '/fromLL')
        
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.client_futures = []

        self.get_logger().info('Ready for waypoints...')

    def gps_point_callback(self, msg: Point):
        
    
        wp = latLonYaw2Geopose(msg.point.y, msg.point.x)
        self.req = FromLL.Request()
        self.req.ll_point.longitude = wp.position.longitude
        self.req.ll_point.latitude = wp.position.latitude
        self.req.ll_point.altitude = wp.position.altitude

        self.get_logger().info("Waypoint added to conversion queue...")
        self.client_futures.append(self.localizer.call_async(self.req))
        
        

    def command_send_cb(self, future):
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = future.result().map_point
        dx = pow(self.resp.pose.position.x,2)
        dy = pow(self.resp.pose.position.y,2)
        d = math.sqrt(dx + dy)
        if d < 25:
            self.navigator.goToPose(self.resp)
            return
        angle = math.atan2(self.resp.pose.position.y,self.resp.pose.position.x)
        self.poses = PoseArray()
        self.poses.header._frame_id = 'map'
        self.poses.header.stamp = self.get_clock().now().to_msg()
        for i in range(1,(d/24)+1):
            new_x = 24 * i * math.cos(angle)
            new_y = 24 * i * math.sin(angle)
            pos = Pose()
            pos.position.x = new_x
            pos.position.y = new_y
            self.poses.poses.append(pos)
        self.navigator.goThroughPoses(self.poses)
            
        

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    self.client_futures.remove(f)
                    self.get_logger().info("Following converted waypoint...")
                    self.command_send_cb(f)
                else:
                    incomplete_futures.append(f)
                    
            self.client_futures = incomplete_futures

def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    gps_wpf.spin()


if __name__ == "__main__":
    main()
