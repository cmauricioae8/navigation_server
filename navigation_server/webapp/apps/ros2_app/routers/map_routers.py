#!/usr/bin/env python
import time
from typing import Union
from fastapi import APIRouter, status
from fastapi.requests import Request

from navigation_server.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from navigation_server.webapp.apps.ros2_app.serializers.map_serializers import (
    PointDataSerializer,
    ScanDataSerializer,
    PoseDataSerializer,
    PathPlanDataSerializer,
    MapDataSerializer,
    MapFullDataSerializer,
    MapResponseSerializer,
)
from navigation_server.webapp.apps.ros2_app.forms.general_forms import Mode
from navigation_server.webapp.apps.base.errors import ERRORS
from navigation_server.base_node import base_node

router = APIRouter()


@router.get(
    "/map/",
    response_model=Union[MapResponseSerializer, SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def get_map(
    request: Request,
    mode: Mode = Mode.UNIQUE,
    with_costmap: bool = False,
    with_scan: bool = False,
    with_path_plan: bool = False,
):
    """
    Get map image

    Get map image from the robot, the image is included in the response as a
    base64 string and can be received in three different modes:
    - **unique**: the robot will send the map only once in response (For all modes)
    - **event**: the robot will send the map every time it receives a new one
        (For mapping)
    - **stop**: the robot will stop listening to the map topic (For mapping)
    """

    if mode == Mode.STOP:
        # unsubscribe from all topics and stop send continuously
        base_node.map_subscriber.try_unsubscribe()
        base_node.map_subscriber.send_continuously = False
        return SimpleResponse(status="OK", message="Captura de imagen detenida")

    if base_node.map_subscriber.send_continuously:
        data = MapFullDataSerializer(
            map_data=MapDataSerializer(
                image=base_node.map_subscriber.map_data.image_to_base64(),
                width=base_node.map_subscriber.map_data.width,
                height=base_node.map_subscriber.map_data.height,
                resolution=base_node.map_subscriber.map_data.resolution,
                origin_x=base_node.map_subscriber.map_data.origin_x,
                origin_y=base_node.map_subscriber.map_data.origin_y,
                origin_rad=base_node.map_subscriber.map_data.origin_rad,
            )
        )
        if with_costmap:
            data.costmap_data = MapDataSerializer(
                image=base_node.costmap_subscriber.map_data.image_to_base64(),
                width=base_node.costmap_subscriber.map_data.width,
                height=base_node.costmap_subscriber.map_data.height,
                resolution=base_node.costmap_subscriber.map_data.resolution,
                origin_x=base_node.costmap_subscriber.map_data.origin_x,
                origin_y=base_node.costmap_subscriber.map_data.origin_y,
                origin_rad=base_node.costmap_subscriber.map_data.origin_rad,
            )
        if with_scan:
            data.scan_data = ScanDataSerializer(
                length=base_node.scan_subscriber.scan_data.length,
                points=[
                    PointDataSerializer(
                        x=point.x,
                        y=point.y,
                    )
                    for point in base_node.scan_subscriber.scan_data.points
                ],
            )
        if with_path_plan:
            data.path_plan_data = PathPlanDataSerializer(
                length=base_node.path_plan_subscriber.path_plan_data.length,
                poses=[
                    PoseDataSerializer(
                        position_x=pose.position_x,
                        position_y=pose.position_y,
                        orientation=pose.orientation,
                    )
                    for pose in base_node.path_plan_subscriber.path_plan_data.poses
                ],
            )
        return MapResponseSerializer(status="OK", message="Imagen recibida", data=data)
    # always front map

    # subscribe to topics
    base_node.map_subscriber.try_subscribe()
    if with_costmap:
        base_node.costmap_subscriber.try_subscribe()
    if with_scan:
        base_node.scan_subscriber.try_subscribe()
    if with_path_plan:
        base_node.path_plan_subscriber.try_subscribe()

    # wait for map message
    t0 = time.time()
    while not base_node.map_subscriber.map_available and time.time() - t0 < 5.0:
        time.sleep(0.2)
    # wait for costmap message
    if with_costmap:
        t0 = time.time()
        while not base_node.costmap_subscriber.map_available and time.time() - t0 < 5.0:
            time.sleep(0.2)
    # wait for lidar message
    if with_scan:
        t0 = time.time()
        while not base_node.scan_subscriber.scan_available and time.time() - t0 < 5.0:
            time.sleep(0.2)
    # wait for path message
    if with_path_plan:
        t0 = time.time()
        while (
            not base_node.path_plan_subscriber.path_plan_available
            and time.time() - t0 < 5.0
        ):
            time.sleep(0.2)

    # if map available
    if base_node.map_subscriber.map_available:
        data = MapFullDataSerializer(
            map_data=MapDataSerializer(
                image=base_node.map_subscriber.map_data.image_to_base64(),
                width=base_node.map_subscriber.map_data.width,
                height=base_node.map_subscriber.map_data.height,
                resolution=base_node.map_subscriber.map_data.resolution,
                origin_x=base_node.map_subscriber.map_data.origin_x,
                origin_y=base_node.map_subscriber.map_data.origin_y,
                origin_rad=base_node.map_subscriber.map_data.origin_rad,
            )
        )
        if with_costmap:
            data.costmap_data = MapDataSerializer(
                image=base_node.costmap_subscriber.map_data.image_to_base64(),
                width=base_node.costmap_subscriber.map_data.width,
                height=base_node.costmap_subscriber.map_data.height,
                resolution=base_node.costmap_subscriber.map_data.resolution,
                origin_x=base_node.costmap_subscriber.map_data.origin_x,
                origin_y=base_node.costmap_subscriber.map_data.origin_y,
                origin_rad=base_node.costmap_subscriber.map_data.origin_rad,
            )
        if with_scan:
            data.scan_data = ScanDataSerializer(
                length=base_node.scan_subscriber.scan_data.length,
                points=[
                    PointDataSerializer(
                        x=round(point.x, 2),
                        y=round(point.y, 2),
                    )
                    for point in base_node.scan_subscriber.scan_data.points
                ],
            )
        if with_path_plan:
            data.path_plan_data = PathPlanDataSerializer(
                length=base_node.path_plan_subscriber.path_plan_data.length,
                poses=[
                    PoseDataSerializer(
                        position_x=round(pose.position_x, 2),
                        position_y=round(pose.position_y, 2),
                        orientation=round(pose.orientation, 2),
                    )
                    for pose in base_node.path_plan_subscriber.path_plan_data.poses
                ],
            )

        response = MapResponseSerializer(
            status="OK", message="Imagen recibida", data=data
        )
    else:
        response = ErrorResponse(
            status="FAIL",
            message="No se recibió imagen",
            error=ERRORS.NO_MAP_DATA.value,
        )

    # unsubscribe from topics
    if mode == Mode.UNIQUE:
        base_node.map_subscriber.try_unsubscribe()
        base_node.map_subscriber.send_continuously = False
        base_node.costmap_subscriber.try_unsubscribe()
        base_node.costmap_subscriber.send_continuously = False
        base_node.scan_subscriber.try_unsubscribe()
        base_node.scan_subscriber.send_continuously = False
        base_node.path_plan_subscriber.try_unsubscribe()
        base_node.path_plan_subscriber.send_continuously = False

    elif mode == Mode.EVENT:
        base_node.map_subscriber.send_continuously = True
        base_node.costmap_subscriber.send_continuously = True
        base_node.scan_subscriber.send_continuously = True
        base_node.path_plan_subscriber.send_continuously = True

    # return data
    return response
