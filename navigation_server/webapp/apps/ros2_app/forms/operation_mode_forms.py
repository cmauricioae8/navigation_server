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
