import os
import cv2 as cv
from math import cos, sin
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParameters
# from rcl_interfaces.msg import Parameter

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
from navigation_server.topics.rotate2person_cmd_publisher import Rotate2PersonCmdPublisher
from navigation_server.topics.n_people_tracked_subscriber import NoPeopleTrackedSubscriber


from navigation_server.clients.navstack_client import NavStackClient
from .utils import image_to_base64_str, from_m_to_px


class BaseNode(Node):
    def __init__(self):
        # Initialize ROS 2 client library
        rclpy.init()
        
        # Create the node
        super().__init__("server_node")
        self.logger = self.get_logger()
        self.logger.info("Starting node BaseNode ...")

        ################################################################################
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

        self.initialpose_publisher = InitialPosePublisher(self, "/initialpose", "geometry_msgs/PoseWithCovarianceStamped")

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

        self.rotate2person_cmd_publisher = Rotate2PersonCmdPublisher(
            self, "/rotate_toward_person_command", "example_interface_msgs/Bool")
        
        self.n_people_tracked_subscriber = NoPeopleTrackedSubscriber(
            self, "/number_people_tracked", "example_interface_msgs/Int16", -1)
        # ros2 topic pub --once /number_people_tracked example_interfaces/msg/Int16 data:\ 2

        self.navstack_client = NavStackClient(self)
        self.stop_waypoints = []

        ## Timer to generate edited map with information
        self.map_layers_timer = self.create_timer(0.5, self.generate_map_layers)
        self.map_layers_timer.cancel()


        ## ROS params declaration of base_node
        self.declare_parameter('nav_distance_tol', 0.25)
        self.declare_parameter('nav_orientation_tol', 0.3)
        self.declare_parameter('action_in_paused', False)


        # Create service clients for getting and setting parameters
        self.octysafe_get_param_cli = self.create_client(GetParameters, '/octy_safe_motion/get_parameters')
        self.octysafe_set_param_cli = self.create_client(SetParameters, '/octy_safe_motion/set_parameters')

        ## List of ROS params to be tracked
        self.octysafe_get_request = GetParameters.Request()
        default_double_value = 0.0
        self.octysafe_get_request.names = ['max_vel_x','min_vel_x','max_vel_theta','manual_vel_gain']
        self.octysafe_params = dict.fromkeys(self.octysafe_get_request.names, default_double_value)

        self.octysafe_set_request = SetParameters.Request()
        

        self.logger.info("... BaseNode initialized")

        emitEvent(
            "on_status_change",
            {
                "data": {
                    "general": {"ready": True}
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
        self.rotate2person_cmd_publisher.try_create_publisher()
        self.n_people_tracked_subscriber.try_subscribe()
        self.logger.info("Topics initialized")
    

    def generate_map_layers(self):
        if self.map_subscriber is not None:
            if self.map_subscriber.map_available and len(self.map_subscriber.map_data.data) > 0:
                base_map_img = self.map_subscriber.getMapImageFromData()
                height, resolution = self.map_subscriber.map_data.height, self.map_subscriber.map_data.resolution
                o_x, o_y = self.map_subscriber.map_data.origin_x, self.map_subscriber.map_data.origin_y

                #Add costmap is available
                if self.costmap_subscriber.map_available and len(self.costmap_subscriber.map_data.data) > 0:
                    costmap_img = self.costmap_subscriber.getMapImageFromData()
                    base_map_img = base_map_img + costmap_img

                #Add path data
                if self.path_plan_subscriber.path_plan_available:
                    n_points = len(self.path_plan_subscriber.path_plan_data.poses)
                    for i in range(n_points-1):
                        p = self.path_plan_subscriber.path_plan_data.poses[i]
                        pf = self.path_plan_subscriber.path_plan_data.poses[i+1]

                        px_x = from_m_to_px(p.position_x - o_x, resolution)
                        px_y = from_m_to_px(p.position_y - o_y, resolution, True, height)

                        pxf_x = from_m_to_px(pf.position_x - o_x, resolution)
                        pxf_y = from_m_to_px(pf.position_y - o_y, resolution, True, height)
                        
                        # cv.circle(base_map_img, (px_x, px_y), 2, (0,200,0), -1)
                        cv.line(base_map_img, (px_x, px_y), (pxf_x, pxf_y), (0,200,0), 1)
                
                #Add robot
                if self.amcl_pose_subscriber.pose_available:
                    pos_x = self.amcl_pose_subscriber.pose_data.position_x
                    pos_y = self.amcl_pose_subscriber.pose_data.position_y
                    orientation = self.amcl_pose_subscriber.pose_data.orientation

                    pos_xf, pos_yf = pos_x + 0.5*cos(orientation), pos_y + 0.5*sin(orientation)

                    pixel_x = from_m_to_px(pos_x - o_x, resolution)
                    pixel_y = from_m_to_px(pos_y - o_y, resolution, True, height)
                    pixel_xf = from_m_to_px(pos_xf - o_x, resolution)
                    pixel_yf = from_m_to_px(pos_yf - o_y, resolution, True, height)

                    cv.circle(base_map_img, (pixel_x, pixel_y), 7, (0,0,235), -1)
                    cv.line(base_map_img, (pixel_x, pixel_y), (pixel_xf, pixel_yf), (0,0,235), 3)

                #Add LiDAR data
                if self.scan_subscriber.scan_available:
                    for p in self.scan_subscriber.scan_data.points:
                        #Compensate robot orientation
                        pr_x = p.x * cos(orientation) - p.y * sin(orientation)
                        pr_y = p.x * sin(orientation) + p.y * cos(orientation)
                        
                        #Compensate robot position
                        px_x = from_m_to_px(pr_x + pos_x - o_x, resolution)
                        px_y = from_m_to_px(pr_y + pos_y - o_y, resolution, True, height)
                        
                        cv.circle(base_map_img, (px_x, px_y), 2, (255,0,0), -1)

                maplayers_dict = {"data": {"image": image_to_base64_str(base_map_img)}}
                emitEvent("maplayers", maplayers_dict)


    def octysafe_get_params(self):
        if not self.octysafe_get_param_cli.wait_for_service(timeout_sec=1.0):
            return False
        
        future = self.octysafe_get_param_cli.call_async(self.octysafe_get_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result(): #1=bool, 2=integer, 4=string
            for i, param_value in enumerate(future.result().values):
                if param_value.type == 3:
                    double_param = param_value.double_value
                    parameter_name = self.octysafe_get_request.names[i]
                    self.octysafe_params[parameter_name] = double_param # update dict
        else:
            self.logger().error('Failed to call octy_safe_motion get_parameters service')
            return False
        
        return True
    

    def octysafe_set_params(self):
        pass


base_node = BaseNode()
