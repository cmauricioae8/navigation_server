#!/usr/bin/env python
from typing import Union
from fastapi import APIRouter, status

from navigation_server.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from navigation_server.webapp.apps.ros2_app.forms.operation_mode_forms import OperationModeRequestForm
from navigation_server.webapp.apps.ros2_app.serializers.operation_mode_serializers import (
    OperationModeSerializer,
    OperationModeResponseSerializer,
)

from navigation_server.modules.mode_manager import mode_manager

router = APIRouter()


@router.get(
    "/operation_mode/",
    response_model=OperationModeResponseSerializer,
    status_code=status.HTTP_200_OK,
)
def get_operation_mode():
    """
    Get current operation mode

    Get the current operation mode from the robot.
    """
    return OperationModeResponseSerializer(
        status="OK",
        message="Modo de operación actual",
        data=OperationModeSerializer(
            mode=mode_manager.mode,
            ready=mode_manager.mode_ready,
        ),
    )


@router.post(
    "/operation_mode/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def set_operation_mode(form_data: OperationModeRequestForm):
    """
    Set operation mode

    Set the operation mode in the robot. The mode can be: static, teleoperation,
    mapping, waypoint and navigation (with keepout and speed maps).
    """

    status, response = mode_manager.set_mode(
        form_data.mode,
        map_id=form_data.map_id,
    )

    if status:
        return SimpleResponse(status="OK", message="Modo de operación solicitado")
    else:
        return ErrorResponse(
            status="ERROR",
            message="Error al solicitar el modo de operación",
            error=response,
        )
