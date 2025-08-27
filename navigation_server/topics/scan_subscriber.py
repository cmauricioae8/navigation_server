#!/usr/bin/env python

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .base_topics import BaseSubscriber
from navigation_server.topics.data_types import PointData
from navigation_server.webapp.socket_io import emitEvent


class ScanData:
    def __init__(self):
        self.length: int = 0
        self.points: list[PointData] = []
        self.min_distance: float = 0
        self.min_angle: int = 0

    def update(self, points: list[PointData], min_distance: float, min_angle: float):
        self.length = len(points)
        self.points = points
        self.min_distance = min_distance
        self.min_angle = int(min_angle * 180 / math.pi)

    def to_dict(self) -> dict:
        return {
            "length": self.length,
            "points": [point.to_dict() for point in self.points],
            "min_distance": self.min_distance,
            "min_angle": self.min_angle,
        }


class ScanSubscriber(BaseSubscriber):
    def __init__(
        self,
        node: Node,
        topic_name: str,
        message_type: str,
        max_rate: int = -1,
        max_density: float = -1,
        rotation_angle: float = 0,
        offset_x: int = 0,
        offset_y: int = 0,
        qos_profile=1,
    ):
        super().__init__(node, topic_name, message_type, max_rate, qos_profile)

        # attributes for pose topic
        self.max_density = max_density
        self.rotation_angle = rotation_angle
        self.offset_x = offset_x
        self.offset_y = offset_y

        # state for pose topic
        self.scan_data: ScanData = ScanData()
        self.scan_available = False

        # assign message class
        self.message_class = LaserScan

    # override on subscribed
    def on_subscribed(self):
        self.scan_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        self.scan_available = False

    # override safe_callback
    def safe_callback(self, msg):
        points = []
        min_distance = 999
        min_angle = 0
        
        if len(msg.ranges) == 0:
            self.scan = None
            self.processing_message = False
            return
        angle = msg.angle_min
        last_angle = msg.angle_min
        for dist in msg.ranges:
            if dist < msg.range_min or dist > msg.range_max:
                angle += msg.angle_increment
                continue
            if self.max_density > 0 and abs(angle - last_angle) < self.max_density:
                angle += msg.angle_increment
                continue
            rot_angle = angle + self.rotation_angle
            if dist < min_distance:
                min_distance = dist
                min_angle = rot_angle
            x = dist * math.cos(rot_angle)
            y = dist * math.sin(rot_angle)
            points.append(PointData(x, y))
            last_angle = angle
            angle += msg.angle_increment

        if min_distance == 999:
            min_distance = 0

        self.scan_data.update(points, min_distance, min_angle)
        self.scan_available = len(points) > 0

        # emit event
        if self.scan_available:
            emitEvent("scan", self.scan_data.to_dict())
