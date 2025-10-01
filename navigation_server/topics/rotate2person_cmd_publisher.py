#!/usr/bin/env python

from rclpy.node import Node
from .base_topics import BasePublisher
from example_interfaces.msg import Bool


class Rotate2PersonCmdPublisher(BasePublisher):
    def __init__(self, node: Node, topic_name: str, message_type: str):
        super().__init__(node, topic_name, message_type)

        self.message_class = Bool
        self.pub_data = Bool()

    def publish(self, data: bool):
        if self.publisher is None:
            return
        self.pub_data.data = data
        self.publisher.publish(self.pub_data)
