
from typing import Union
from fastapi import APIRouter, status

from navigation_server.webapp.apps.waypoints.cruds.waypoint_cruds import (
    waypoint_crud,
    validate_name_exists
)
from navigation_server.webapp.apps.waypoints.models import Waypoint
from navigation_server.webapp.apps.navigation.forms.navigation_forms import (
    NavigationRequestForm,
    DeliveryRequestForm,
    NavAdminControlRequestForm
)
from navigation_server.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from navigation_server.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationResponseSerializer,
)
from navigation_server.webapp.apps.base.errors import ERRORS
from navigation_server.modules.navigation_manager import navigation_manager
from navigation_server.modules.navigation_manager_models import PathMode
from navigation_server.modules.mode_manager import mode_manager
from navigation_server.modules.mode_manager import OperationMode

router = APIRouter()


@router.get(
    "/",
    response_model=NavigationResponseSerializer,
    status_code=status.HTTP_200_OK,
)
def get_navigation():
    """
    Navigation status

    Get the current navigation status.
    """
    return NavigationResponseSerializer(
        status="OK",
        message="Current navigation status",
        data=navigation_manager.get_navigation_status(),
    )


@router.post(
    "/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def set_navigation(form_data: NavigationRequestForm):
    """
    Navigation request

    Set the navigation mode and path. Also can loop, stop, pause, resume or cancel the
    current navigation.
    """

    if mode_manager.mode != OperationMode.NAVIGATION:
        return ErrorResponse(
            status="FAIL",
            message="Nothing to do, Robot is not in navigation mode",
            error=ERRORS.NO_AVAILABLE_IN_MODE,
        )

    if not mode_manager.mode_ready:
        return ErrorResponse(
            status="FAIL",
            message="System error",
            error=ERRORS.MODE_NOT_READY,
        )

    # For mode: stop
    if form_data.mode == PathMode.STOP:
        if navigation_manager.on_navigation:
            navigation_manager.admin_pause = False
            navigation_manager.cancel_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation stopped",
            )
        else:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )

    # For mode: pause
    elif form_data.mode == PathMode.PAUSE:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )
        elif navigation_manager.paused_navigation or navigation_manager.admin_pause: #----
            return SimpleResponse(
                status="OK",
                message="Navigation already paused",
            )
        else:
            navigation_manager.pause_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation paused",
            )

    # For mode: resume
    elif form_data.mode == PathMode.RESUME:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )
        elif not navigation_manager.paused_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation already resumed",
            )
        elif navigation_manager.admin_pause: #----------------------
            return SimpleResponse(
                status="FAIL",
                message="Navigation paused by admin",
            )
        else:
            navigation_manager.resume_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation resumed",
            )

    # For mode: cancel_goal
    elif form_data.mode == PathMode.CANCEL_GOAL:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="No goal to cancel",
            )
        else:
            navigation_manager.cancel_goal()
            return SimpleResponse(
                status="OK",
                message="Goal canceled",
            )

    elif navigation_manager.on_navigation:
        return ErrorResponse(
            status="FAIL",
            message="Currently on path",
            error=ERRORS.CURRENTLY_ON_PATH,
        )

    # For mode: loop, reverse_loop, once
    status, response = navigation_manager.start_navigation(
        form_data.path_id, form_data.waypoint_id, form_data.mode, form_data.laps
    )

    if status:
        return SimpleResponse(
            status="OK",
            message="Navigation start set",
        )
    else:
        return ErrorResponse(
            status="FAIL",
            message="Navigation start fail",
            error=response,
        )


@router.post(
    "/admin_control",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def admin_control(form_data: NavAdminControlRequestForm):
    """
    Admin Navigation Control request

    Set the admin navigation control. The possible options are: pause, resume or stop the
    current navigation.
    """

    if mode_manager.mode != OperationMode.NAVIGATION:
        return ErrorResponse(
            status="FAIL",
            message="Nothing to do, Robot is not in navigation mode",
            error=ERRORS.NO_AVAILABLE_IN_MODE,
        )

    if not mode_manager.mode_ready:
        return ErrorResponse(
            status="FAIL",
            message="System error",
            error=ERRORS.MODE_NOT_READY,
        )

    # For action: stop
    if form_data.action == PathMode.STOP:
        if navigation_manager.on_navigation:
            navigation_manager.admin_pause = False
            navigation_manager.cancel_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation stopped",
            )
        else:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )

    # For action: pause
    elif form_data.action == PathMode.PAUSE:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )
        elif navigation_manager.paused_navigation or navigation_manager.admin_pause:
            return SimpleResponse(
                status="OK",
                message="Navigation already paused",
            )
        else:
            navigation_manager.admin_pause = True
            navigation_manager.pause_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation paused by Admin",
            )

    # For action: resume
    elif form_data.action == PathMode.RESUME:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation already stopped",
            )
        elif not navigation_manager.paused_navigation and not navigation_manager.admin_pause: #---
            return SimpleResponse(
                status="OK",
                message="Navigation already resumed",
            )
        else:
            navigation_manager.admin_pause = False
            navigation_manager.resume_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation resumed",
            )


@router.post(
    "/delivery/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def delivery_test(form_data: DeliveryRequestForm):
    """
    Delivery

    Receives either the WP id or the WP name to send the robot to that goal
    """

    if mode_manager.mode != OperationMode.NAVIGATION:
        return ErrorResponse(
            status="FAIL",
            message="Nothing to do, Robot is not in navigation mode",
            error=ERRORS.NO_AVAILABLE_IN_MODE,
        )

    if not mode_manager.mode_ready:
        return ErrorResponse(
            status="FAIL",
            message="System error",
            error=ERRORS.MODE_NOT_READY,
        )
    
    if form_data.waypoint_id is not None: # If wp id is given
        status, response = navigation_manager.start_navigation(
            0, form_data.waypoint_id, 'once', 0
        )
    elif form_data.waypoint_name is not None: # If name is given, first get the wp id
        if validate_name_exists(form_data.waypoint_name, mode_manager.map_id):
            wp_result = waypoint_crud.get_by_fields(
                {"name": form_data.waypoint_name, "map_id": mode_manager.map_id}
            )
            wp_id = wp_result.id
            status, response = navigation_manager.start_navigation(0, wp_id, 'once', 0)
        else:
            return ErrorResponse(
                status="FAIL",
                message="Navigation start fail",
                error=ERRORS.WAYPOINT_DOES_NOT_EXIST
            )
    else:
        return ErrorResponse(
            status="FAIL", message="Waypoint either Id or name not provided",
            error="Fail")


    if status:
        return SimpleResponse(
            status="OK",
            message="Navigation start set",
        )
    else:
        return ErrorResponse(
            status="FAIL",
            message="Navigation start fail",
            error=response,
        )
