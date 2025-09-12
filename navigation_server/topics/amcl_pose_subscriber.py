#!/usr/bin/env python

import time
from rclpy.node import Node
from geometry_msgs.msg import Pose

from .base_topics import BaseSubscriber
from ..utils import euler_from_quaternion
from navigation_server.webapp.socket_io import emitEvent


class RobotPoseData:
    def __init__(self):
        self.position_x: float = 0.0
        self.position_y: float = 0.0
        self.orientation: float = 0.0
        self.covariance: list = []
        self.timestamp: float = time.time()

    def update(
        self,
        position_x: float,
        position_y: float,
        orientation: float,
        covariance: list,
    ):
        self.position_x = position_x
        self.position_y = position_y
        self.orientation = orientation
        self.covariance = covariance
        self.timestamp = time.time()

    def to_dict(self) -> dict:
        return {
            "position_x": round(self.position_x, 2),
            "position_y": round(self.position_y, 2),
            "orientation": round(self.orientation, 2),
            "timestamp": self.timestamp,
        }


class AmclPoseSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for pose topic. State for pose topic
        self.pose_data = RobotPoseData()
        self.pose_available = False
        self.message_class = Pose

    # override on subscribed
    def on_subscribed(self):
        self.pose_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        self.pose_available = False

    # override safe_callback
    def safe_callback(self, msg):
        position_x = msg.position.x
        position_y = msg.position.y
        orientation = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )[2]
        covariance = []

        self.pose_data.update(position_x, position_y, orientation, covariance)
        self.pose_available = True
        # emitEvent("robot_pose", self.pose_data.to_dict())
