#!/usr/bin/env python
from typing import Union
from fastapi import APIRouter, status
from fastapi.requests import Request

from navigation_server.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from navigation_server.webapp.apps.ros2_app.forms.set_pose_forms import (
    SetPoseRequestForm,
    SetPoseToWaypointRequestForm,
)
from navigation_server.webapp.apps.waypoints.cruds.waypoint_cruds import waypoint_crud
from navigation_server.webapp.settings import OperationMode
from navigation_server.webapp.apps.base.errors import ERRORS

from navigation_server.base_node import base_node
from navigation_server.modules.mode_manager import mode_manager

router = APIRouter()


@router.post(
    "/set_pose/free/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Set Initial Pose",
)
def set_pose(request: Request, form: SetPoseRequestForm):
    """
    Set the robot pose in the map. Publish on 'initialpose' topic. The position is 
    given in meters and the orientation in radians. Three fields MUST be provided.
    """
    if not mode_manager.mode_ready or mode_manager.mode not in (
        OperationMode.TELEOPERATION,
        OperationMode.WAYPOINTS,
        OperationMode.NAVIGATION,
    ):
        return ErrorResponse(
            status="FAIL",
            message="No se puede setear la posición en el estado actual",
            error=ERRORS.NO_AVAILABLE_IN_MODE.value,
        )

    status, response = base_node.initialpose_publisher.set_pose(
        form.position_x, form.position_y, form.orientation
    )

    if status:
        return SimpleResponse(status="OK", message=response)
    else:
        return ErrorResponse(status="FAIL", message=response, error="")


@router.post(
    "/set_pose/waypoint/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Set Pose From a WP",
)
def set_pose_to_waypoint(request: Request, form: SetPoseToWaypointRequestForm):
    """
    Set robot pose from a waypoint id

    Set the robot pose in the map from a waypoint.
    """
    if not mode_manager.mode_ready or mode_manager.mode not in (
        OperationMode.TELEOPERATION,
        OperationMode.WAYPOINTS,
        OperationMode.NAVIGATION,
    ):
        return ErrorResponse(
            status="FAIL",
            message="No se puede setear la posición en el estado actual",
            error=ERRORS.NO_AVAILABLE_IN_MODE.value,
        )

    waypoint = waypoint_crud.get(form.waypoint_id)

    status, response = base_node.initialpose_publisher.set_pose_to_waypoint(waypoint)

    if status:
        return SimpleResponse(status="OK", message=response)
    else:
        return ErrorResponse(status="FAIL", message=response, error="")
