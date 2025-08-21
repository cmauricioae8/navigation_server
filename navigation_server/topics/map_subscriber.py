#!/usr/bin/env python

import cv2 as cv
import numpy as np
import copy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from .base_topics import BaseSubscriber
from ..utils import image_to_base64_str
# from navigation_server.webapp.socket_io import emitEvent


class MapColors:
    def __init__(
        self,
        WALL: list[int] = [140, 142, 144],
        FLOOR: list[int] = [76, 102, 101],
        UNKNOWN: list[int] = [24, 28, 35],
        ALMOST_WALL: list[int] = [140, 70, 70],
        ALMOST_FLOOR: list[int] = [76, 102, 101],
    ):
        # Color mapping for the map in RGB format, [R,G,B]. Range 0-255
        self.WALL: list[int] = WALL  # Color for the wall
        self.FLOOR: list[int] = FLOOR  # Color for the floor
        self.UNKNOWN: list[int] = UNKNOWN  # Color for the unknown
        self.ALMOST_WALL: list[int] = ALMOST_WALL  # Color for the almost wall
        self.ALMOST_FLOOR: list[int] = ALMOST_FLOOR  # Color for the almost floor

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

class MapData:
    def __init__(self):
        self.image: np.ndarray = None
        self.data: list = []
        self.width: int = 0
        self.height: int = 0
        self.resolution: float = 0
        self.origin_x: float = 0
        self.origin_y: float = 0
        self.origin_rad: float = 0

    def update(
        self,
        data: list,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_rad: float,
    ):
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_rad = origin_rad

    def image_to_base64(self) -> str:
        return image_to_base64_str(self.image)

    def to_dict(self) -> dict:
        return {
            "image": self.image_to_base64(),
            "width": self.width,  # in pixels
            "height": self.height,  # in pixels
            "resolution": self.resolution,  # m/pixel
            "origin_x": self.origin_x,  # m
            "origin_y": self.origin_y,  # m
            "origin_rad": self.origin_rad,  # rad
        }


class MapSubscriber(BaseSubscriber):
    def __init__(
        self,
        node: Node,
        topic_name: str,
        max_rate: int = -1,
        is_costmap: bool = False,
        event_name: str = "map",
    ):
        super().__init__(node, topic_name, "nav_msgs/OccupancyGrid", max_rate)

        # attributes for map topic
        if is_costmap:
            COSTMAP_COLORS = copy.deepcopy(MapColors())
            COSTMAP_COLORS.FLOOR = [0, 0, 0]
            self.colors = COSTMAP_COLORS
        else:
            self.colors = MapColors()

        self.event_name = event_name

        # state for map topic
        self.map_data: MapData = MapData()
        self.map_available = False

        # assign message class
        self.message_class = OccupancyGrid

    def clear_map(self):
        self.map_available = False

    def safe_callback(self, msg: OccupancyGrid):
        # self.node.logger.info(f"Map received from '{self.topic_name}' topic")
        # print("Received a map of size %d x %d" % (msg.info.width, msg.info.height))
        # print("Resolution: %.2f m/pix" % (msg.info.resolution))
        # print("Origin: %.2f, %.2f, %.2f" % (
        #  msg.info.origin.position.x,
        #  msg.info.origin.position.y,
        #  msg.info.origin.position.z))

        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        # That is the coordinate of the lower left corner of your map in the reference
        # frame
        origin_x = abs(msg.info.origin.position.x)
        origin_y = msg.info.height * msg.info.resolution - abs(msg.info.origin.position.y)
        origin_rad = msg.info.origin.position.z

        self.map_data.update(
            list(msg.data),
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            origin_x,
            origin_y,
            origin_rad,
        )

        if len(self.map_data.data) > 0:
            self.map_data.image = self.getMapImageFromData()
            # emitEvent(self.event_name, {"data": self.map_data.to_dict()})

        self.map_available = len(self.map_data.data) > 0

    def add_layer(self, layer_data: list):
        if len(layer_data) == 0:
            return

        if len(layer_data) == len(self.map_data.data):
            for i in range(len(layer_data)):
                # continue if cell is unknown on destination map
                if self.map_data.data[i] == -1:
                    continue
                # continue if cell is unknown or floor on source map
                if layer_data[i] <= 0:
                    continue
                # add the layer data to the map data
                self.map_data.data[i] = layer_data[i]
        # print()
        # print("Finish adding layer")

    def getMapImageFromData(self):
        if len(self.map_data.data) == 0:
            return None
        try:
            # Convert the map data to a numpy array with color values
            data = list(map(self.type_cell_to_color, self.map_data.data))
            img = np.array(data, dtype=np.uint8)
            img = np.reshape(img, (self.map_data.height, self.map_data.width, 3))
            # flip the map because the map is mirrored
            img = cv.flip(img, 1)
            # rotate the map because the map is rotated 180 degrees
            img = cv.rotate(img, cv.ROTATE_180)
        except Exception as e:
            self.node.logger.error("ERROR: while transform map into openCV type: " + str(e))
            img = None

        return img

    def type_cell_to_color(self, type_cell: int):
        if type_cell == -1:  # unknown
            return self.colors.UNKNOWN[::-1]
        elif type_cell == 0:  # floor
            return self.colors.FLOOR[::-1]
        elif type_cell == 100:  # wall
            return self.colors.WALL[::-1]
        elif 0 < type_cell < 100:
            almost_wall = self.colors.ALMOST_WALL
            almost_floor = self.colors.ALMOST_FLOOR
            return (
                int(
                    almost_floor[2]
                    + (almost_wall[2] - almost_floor[2]) * (type_cell - 1) / 98
                ),
                int(
                    almost_floor[1]
                    + (almost_wall[1] - almost_floor[1]) * (type_cell - 1) / 98
                ),
                int(
                    almost_floor[0]
                    + (almost_wall[0] - almost_floor[0]) * (type_cell - 1) / 98
                ),
            )
        else:
            # print("Unknown type cell: %d" % type_cell)
            return (0, 0, 0)
