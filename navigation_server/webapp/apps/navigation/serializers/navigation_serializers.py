
from datetime import datetime
from pydantic import Field

from navigation_server.webapp.apps.waypoints.serializers.waypoint_serializers import (
    WaypointSimplestSerializer,
)
from navigation_server.webapp.apps.paths.serializers.path_serializers import (
    PathSimplestSerializer,
)
from navigation_server.webapp.apps.base.serializers import DataResponse, BaseSerializer
from navigation_server.modules.navigation_manager_models import NavigationState, PathMode


class NavigationStatusSerializer(BaseSerializer):
    on_navigation: bool = Field(
        description=(
            "'True' si se encuentra actualmente en navegación, de lo contrario"
            " es 'False'"
        )
    )
    paused_navigation: bool = Field(
        description="'True' si la navegación está pausada, de lo contrario es 'False'"
    )
    path: PathSimplestSerializer | None = Field(description="trayectoria actual")
    mode: PathMode = Field(
        description="Modo de la trayectoria",
    )
    laps: int | None = Field(
        default=None,
        description="Número de repeticiones de la trayectoria, '-1' si es en bucle",
    )
    lap: int = Field(description="Indica la vuelta actual")
    on_waypoint: bool = Field(
        description=(
            "'True' si actualmente se encuentra ubicado en un punto de la "
            "trayectoria, de lo contrario es 'False'"
        )
    )
    next_waypoint: WaypointSimplestSerializer | None = Field(
        description="Siguiente punto de la trayectoria"
    )
    attempt: int = Field(description="Número de intentos en el punto actual (si aplica)")
    start_time: datetime = Field(description="Hora de inicio del punto actual")
    state: NavigationState = Field(
        description="Estado de la meta actual",
    )
    message: str = Field(
        description="Mensaje de estado de la meta actual",
    )


class NavigationResponseSerializer(DataResponse):
    data: NavigationStatusSerializer
