import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from navigation_server.webapp.socket_io import emitEvent

from navigation_server.topics.cmd_vel_publisher import CmdVelPublisher
from navigation_server.topics.initialpose_publisher import InitialPosePublisher
from navigation_server.topics.amcl_pose_subscriber import AmclPoseSubscriber
from navigation_server.topics.scan_subscriber import ScanSubscriber
from navigation_server.topics.path_plan_subscriber import PathPlanSubscriber
from navigation_server.topics.map_subscriber import MapSubscriber
from navigation_server.topics.mode_status_publisher import ModeStatusPublisher
from navigation_server.topics.notification_subscriber import NotificationSubscriber
from navigation_server.topics.battery_subscriber import BatterySubscriber

from navigation_server.clients.navstack_client import NavStackClient
# from navigation_server.clients.navigation_client import NavigationClient


class BaseNode(Node):
    def __init__(self):
        # Initialize ROS 2 client library
        rclpy.init()
        
        # Create the node
        super().__init__("server_node")
        self.logger = self.get_logger()
        self.logger.info("Starting node BaseNode ...")

        ################################################################################

        # environ = os.environ.copy()

        lidar_rotation_angle = 0
        lidar_offset_x = 0.3
        lidar_offset_y = 0

        LIDAR_MODEL=os.environ['LIDAR_MODEL']
        self.logger.info(" LIDAR_MODEL: " + LIDAR_MODEL)
        if LIDAR_MODEL == 'rplidar': # otherwise 0 as default
            lidar_rotation_angle = 3.1415

        ROBOT_MODEL=os.environ['ROBOT_MODEL']
        self.logger.info(" ROBOT_MODEL: " + ROBOT_MODEL)
        # This value is given in the ROBOT_MODEL_urdf.xacro file
        if ROBOT_MODEL == 'minibase': lidar_offset_x = 0.24
        elif ROBOT_MODEL == 'pickin' or ROBOT_MODEL == 'octybot': lidar_offset_x = 0.3              
        
        self.cmd_vel_publisher = CmdVelPublisher( self, "/cmd_vel", "geometry_msgs/Twist")

        self.initialpose_publisher = InitialPosePublisher(
            self, "/initialpose", "geometry_msgs/PoseWithCovarianceStamped"
        )

        self.amcl_pose_subscriber = AmclPoseSubscriber( self, "/amcl_robot_pose", "geometry_msgs/Pose", 5)

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
            self, "/notifications", "rcl_interfaces/Log", -1
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
        # ros2 topic pub --rate 5 /voltage std_msgs/msg/Float64 data:\ 24.1

        self.navstack_client = NavStackClient(self)
        # self.navigation_client = NavigationClient(self)
        self.stop_waypoints = []

        self.logger.info("... BaseNode initialized")

        emitEvent(
            "on_status_change",
            {
                "data": {
                    "general": {
                        "ready": True,
                    }
                }
            },
        )


    def init_topics(self):
        # Try to subscribe and create the publishers for initial topics
        self.logger.info("Initializing topics ...")
        
        self.cmd_vel_publisher.try_create_publisher()
        self.initialpose_publisher.try_create_publisher()
        self.mode_status_publisher.try_create_publisher()        
        self.notifications_subscriber.try_subscribe()
        self.battery_subscriber.try_subscribe()
        self.navstack_client.try_create_client()
        # self.navigation_client.try_create_client()
        self.logger.info("Topics initialized")


base_node = BaseNode()
