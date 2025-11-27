#!/usr/bin/env python

import logging
from rclpy.node import Node
from rcl_interfaces.msg import Log
from .base_topics import BaseSubscriber
from navigation_server.webapp.socket_io import emitEvent

logger = logging.getLogger("backend") ## info logger is not displayed in terminal


class NotificationSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # assign message class
        self.message_class = Log

    # override callback
    def callback(self, msg: Log):
        message = msg.msg
        logger.warning(f"***** NOTIFICATIONS: {msg.level} - {msg.name} - {message}")

        emitEvent("notifications", {"msg": message})

        ## Filter notifications for robot status
        notifications_dict = {}

        ## LiDAR
        if message.find('SEN-LID-1-104') != -1:
            notifications_dict['source'] = "lidar"
            notifications_dict['status'] = "FAIL"
            notifications_dict['msg'] = "No hay datos del LiDAR"
        if message.find('SEN-LID-0-0') != -1:
            notifications_dict['source'] = "lidar"
            notifications_dict['status'] = "OK"
            notifications_dict['msg'] = "LiDAR funcionando"
        
        ## Wheels
        # if message.find('COM-RUE-1-C204') != -1 or message.find('COM-RUE-2-C204'):
        #     notifications_dict['source'] = "wheels"
        #     notifications_dict['status'] = "FAIL"
        #     notifications_dict['msg'] = "No hay datos de las ruedas ruedas"
        # if message.find('COM-RUE-0-0') != -1:
        #     notifications_dict['source'] = "wheels"
        #     notifications_dict['status'] = "OK"
        #     notifications_dict['msg'] = "Ruedas funcionando"
        
        ## Blocking
        if message.find('+LDR_F') != -1:
            notifications_dict['source'] = "block"
            notifications_dict['status'] = "FAIL"
            notifications_dict['msg'] = "Espacio frontal obstruido"
        if message.find('-LDR_F')!= -1:
            notifications_dict['source'] = "block"
            notifications_dict['status'] = "OK"
            notifications_dict['msg'] = "Espacio frontal libre"
        
        if message.find('+LDR_L') != -1:
            notifications_dict['source'] = "block"
            notifications_dict['status'] = "FAIL"
            notifications_dict['msg'] = "Espacio lateral obstruido"
        if message.find('-LDR_L')!= -1:
            notifications_dict['source'] = "block"
            notifications_dict['status'] = "OK"
            notifications_dict['msg'] = "Espacio lateral libre"

        emitEvent("notifications", notifications_dict)
