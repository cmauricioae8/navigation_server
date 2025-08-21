#!/usr/bin/env python

import logging

from rclpy.node import Node
from rcl_interfaces.msg import Log
from .base_topics import BaseSubscriber
# from navigation_server.webapp.socket_io import emitEvent

logger = logging.getLogger("backend")


class NotificationSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # assign message class
        self.message_class = Log

    # override callback
    def callback(self, msg: Log):
        level = msg.level
        name = msg.name
        message = msg.msg
        logger.info(f"NOTIFICATIONS: {level} - {name} - {message}")
        # emitEvent("notifications", {"data": message})
