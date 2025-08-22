
from typing import Union
from fastapi import APIRouter, status

from navigation_server.webapp.apps.navigation.forms.navigation_forms import (
    NavigationRequestForm,
)
from navigation_server.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from navigation_server.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationResponseSerializer,
)
from navigation_server.webapp.apps.base.errors import ERRORS
# from navigation_server.modules.navigation_manager import navigation_manager
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

    Set the navigation mode and path. Also can stop, pause, resume or cancel the
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
        elif navigation_manager.paused_navigation:
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
