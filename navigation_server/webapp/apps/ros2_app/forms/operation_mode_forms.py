from pydantic import BaseModel, Field
from navigation_server.webapp.settings import OperationMode


class OperationModeRequestForm(BaseModel):
    mode: OperationMode = Field(
        description="Modo de operación",
        examples=["static", "teleoperation", "mapping", "waypoint", "navigation"],
    )
    map_id: int | None = Field(
        default=None,
        description="ID del mapa",
        examples=[1, 2, 3],
    )
    with_keepout_zones: bool | None = Field(
        default=True,
        description="Con zonas de exclusión",
        examples=[True, False],
    )
    with_speed_limits: bool | None = Field(
        default=True,
        description="Con limites de velocidad",
        examples=[True, False],
    )
    ## TODO: set implicitly with with_keepout_zones and with_speed_limits
