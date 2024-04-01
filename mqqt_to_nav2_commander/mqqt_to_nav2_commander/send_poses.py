
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.duration import Duration
from std_msgs.msg import Bool
import time

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
        self.mqtt_array_pose_sub = self.create_subscription(
            Bool, "/web/follow", self.follow_callback, 1)
        self.update_goal_pub = self.create_publisher(
            PoseStamped,"/goal_update",10
        )
        self.get_logger().info('Ready for waypoints...')
        self.follow_me = False
        self.control = False

    def follow_callback(self,msg: Bool):
        if msg.data:
            self.follow_me = True
        else:
            self.follow_me = False

                
    def gps_point_callback(self, msg: PoseStamped):
        
        if self.follow_me and not self.control:
            self.navigator.goToPose(msg,behavior_tree='/home/andrelorent/tfg_ws/src/hunter_sim_pkg/config/follow_dynamic_waypoint.xml')
            self.control = True
        elif self.follow_me and self.control:
            self.update_goal_pub.publish(msg)
        else:
            self.control = False
            self.navigator.goToPose(msg)
            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print(
                        'Estimated time of arrival: '
                        + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                            / 1e9
                        )
                        + ' seconds.')
                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.navigator.cancelTask()
            # Do something depending on the return code
            result =self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
  

        
    def gps_point_array_callback(self, msg: PoseArray):
        array = []
        for p in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = p
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.navigator.get_clock().now().to_msg()
            array.append(pose_stamped)
            
        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(array)
        i = 0
        while not self.navigator.isTaskComplete():
            i = i+1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(array))
                )
                now = self.navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600.0):
                    self.navigator.cancelTask()
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

            
        
    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            

def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    gps_wpf.spin()


if __name__ == "__main__":
    main()
