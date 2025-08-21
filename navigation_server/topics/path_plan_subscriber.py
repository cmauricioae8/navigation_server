#!/usr/bin/env python

from rclpy.node import Node
from nav_msgs.msg import Path

from .base_topics import BaseSubscriber
from navigation_server.utils import euler_from_quaternion
from navigation_server.topics.data_types import PoseData
from navigation_server.webapp.socket_io import emitEvent


class PathPlanData:
    def __init__(self):
        self.length: int = 0
        self.poses: list[PoseData] = []

    def update(self, poses: list[PoseData]):
        self.length = len(poses)
        self.poses = poses

    def to_dict(self) -> dict:
        return {
            "length": self.length,
            "poses": [pose.to_dict() for pose in self.poses],
        }


class PathPlanSubscriber(BaseSubscriber):
    def __init__(
        self,
        node: Node,
        topic_name: str,
        message_type: str,
        max_rate: int = -1,
        max_density: float = -1,
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for pose topic
        self.max_density = max_density
        # state for pose topic
        self.path_plan_data: PathPlanData = PathPlanData()
        self.path_plan_available = False

        # assign message class
        self.message_class = Path
            

    # override on subscribed
    def on_subscribed(self):
        self.path_plan_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        self.path_plan_available = False

    # override safe_callback
    def safe_callback(self, msg):
        poses = []
        
        if len(msg.poses) == 0:
            self.processing_message = False
            return
        position0 = msg.poses[0].pose.position
        for pose in msg.poses:
            # check if the pose is too close to the previous one
            dist = (
                (pose.pose.position.x - position0.x) ** 2
                + (pose.pose.position.y - position0.y) ** 2
            ) ** 0.5
            # continue if the distance is less than the max density or not the last
            # pose
            if (
                dist < self.max_density
                and pose != msg.poses[0]
                and pose != msg.poses[-1]
            ):
                continue
            position0 = pose.pose.position
            position_x = pose.pose.position.x
            position_y = pose.pose.position.y
            orientation = euler_from_quaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )[2]
            poses.append(PoseData(position_x, position_y, orientation))

        self.path_plan_data.update(poses)
        self.path_plan_available = len(poses) > 0

        if self.path_plan_available:
            emitEvent("path_plan", self.path_plan_data.to_dict())
