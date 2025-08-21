import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from navigation_server.topics.cmd_vel_publisher import CmdVelPublisher
from navigation_server.topics.pose_publisher import PosePublisher
from navigation_server.topics.pose_subscriber import PoseSubscriber
from navigation_server.topics.scan_subscriber import ScanSubscriber
from navigation_server.topics.path_plan_subscriber import PathPlanSubscriber
from navigation_server.topics.map_subscriber import MapSubscriber
from navigation_server.topics.mode_status_publisher import ModeStatusPublisher
from navigation_server.topics.notification_subscriber import NotificationSubscriber
from navigation_server.topics.battery_subscriber import BatterySubscriber


class BaseNode(Node):
    def __init__(self):
        # Initialize ROS 2 client library
        rclpy.init()
        
        # Create the node
        super().__init__("server_node")
        self.logger = self.get_logger()
        self.logger.info("Starting node BaseNode ...")

        ################################################################################

        environ = os.environ.copy()

        lidar_rotation_angle = 0
        lidar_offset_x = 0
        lidar_offset_y = 0

        if "LIDAR_MODEL" in environ:
            self.logger.info(f"LIDAR_MODEL found = {environ['LIDAR_MODEL']}")
            if environ["LIDAR_MODEL"] == "rplidar":
                lidar_rotation_angle = 3.14159
            elif environ["LIDAR_MODEL"] == "pacecat":
                lidar_rotation_angle = 3.14159
            elif environ["LIDAR_MODEL"] == "pacecat_cr":
                lidar_rotation_angle = 0

        if "ROBOT_MODEL" in environ:
            # TODO: Get params from ros params
            if environ["ROBOT_MODEL"] == "octybot":
                lidar_offset_x = 0.0
                lidar_offset_y = 0.0
            elif environ["ROBOT_MODEL"] == "minibase":
                lidar_offset_x = 0.0
                lidar_offset_y = 0.0
            elif environ["ROBOT_MODEL"] == "pickin":
                lidar_offset_x = 0.0
                lidar_offset_y = 0.0
        
        ## TODO: Define lidar_offset_x, lidar_offset_y, lidar_rotation_angle according to ROBOT_MODEL
        
        self.cmd_vel_publisher = CmdVelPublisher( self, "/cmd_vel", "geometry_msgs/Twist")

        self.pose_publisher = PosePublisher(
            self, "/initialpose", "geometry_msgs/PoseWithCovarianceStamped"
        )

        self.pose_subscriber = PoseSubscriber( self, "/amcl_robot_pose", "geometry_msgs/Pose", 5)

        self.scan_subscriber = ScanSubscriber(
            self,
            "/scan",
            "sensor_msgs/LaserScan",
            3,
            0.05,
            lidar_rotation_angle,
            lidar_offset_x,
            lidar_offset_y,
            qos_profile_sensor_data,
        )

        self.path_plan_subscriber = PathPlanSubscriber( self, "/plan", "nav_msgs/Path", -1, 0.5)

        self.map_subscriber = MapSubscriber(
            self,
            "/map",
            -1,
            False,
            event_name="map",
        )
        
        
        self.costmap_subscriber = MapSubscriber(
            self,
            "/global_costmap/costmap",
            -1,
            True,
            event_name="costmap",
        )

        self.mode_status_publisher = ModeStatusPublisher(
            self, "/mode_status", "std_msgs/String"
        )

        self.notifications_subscriber = NotificationSubscriber(
            self, "/notifications", "rcl_interfaces.msg.Log", -1
        )

        self.battery_subscriber = BatterySubscriber(
            self,
            "/voltage",
            "std_msgs/Float64",
            10.0,
            95.0,
            20.0,
            20.0,
            25.0,
            1
        )        

        self.stop_waypoints = []

        self.logger.info("... BaseNode initialized")


    def init_topics(self):
        # Try to subscribe and create the publishers for initial topics
        self.logger.info("Initializing topics ...")
        self.cmd_vel_publisher.try_create_publisher()
        self.pose_publisher.try_create_publisher()
        self.mode_status_publisher.try_create_publisher()        
        self.notifications_subscriber.try_subscribe()
        self.battery_subscriber.try_subscribe()
        self.logger.info("Topics initialized")


base_node = BaseNode()
