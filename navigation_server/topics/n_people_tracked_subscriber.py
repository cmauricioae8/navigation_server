#!/usr/bin/env python

import logging
from rclpy.node import Node
from example_interfaces.msg import Int16
from .base_topics import BaseSubscriber
from navigation_server.webapp.socket_io import emitEvent

logger = logging.getLogger("backend") ## info logger is not displayed in terminal


class NoPeopleTrackedSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # assign message class
        self.message_class = Int16

    # override callback
    def callback(self, msg: Int16):
        message = msg.data
        emitEvent("number_people_detected", {"data": message})
        logger.warning(f"Number of legs detection: {str(message)}")
