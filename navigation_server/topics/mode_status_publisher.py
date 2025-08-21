#!/usr/bin/env python

import logging

from rclpy.node import Node
from std_msgs.msg import String
from .base_topics import BasePublisher

logger = logging.getLogger(__name__)


class ModeStatusPublisher(BasePublisher):
    def __init__(self, node: Node, topic_name: str, message_type: str):
        super().__init__(node, topic_name, message_type)

        # assign message class
        self.message_class = String
        self.pub_data = String()
      

    def publish(self, mode: str):
        if self.publisher is None:
            return
        self.pub_data.data = mode
        self.publisher.publish(self.pub_data)
