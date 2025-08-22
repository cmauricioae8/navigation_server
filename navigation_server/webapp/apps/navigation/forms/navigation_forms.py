
from pydantic import BaseModel, Field

from navigation_server.webapp.apps.navigation.serializers.navigation_serializers import (
    PathMode,
)


class NavigationRequestForm(BaseModel):
    mode: PathMode = Field(
        description="Modo de la trayectoria",
    )
    path_id: int | None = Field(
        default=None,
        description="ID de la trayectoria",
    )
    waypoint_id: int | None = Field(
        default=None,
        description="ID de un punto de interes para navegar a el (delivery)",
    )
    laps: int | None = Field(
        default=None,
        description="Valor: repeticiones de la trayectoria, -1 si bucle infinito",
    )
