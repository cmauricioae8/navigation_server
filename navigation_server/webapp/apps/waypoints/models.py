
from typing import Optional
from sqlmodel import Field

from navigation_server.webapp.apps.base.models import BaseModel


class Waypoint(BaseModel, table=True):
    name: str = Field(max_length=50)
    description: Optional[str] = Field(default="")
    map_id: int = Field(foreign_key="map.id")
    position_x: Optional[float] = Field(default=None)
    position_y: Optional[float] = Field(default=None)
    orientation: Optional[float] = Field(default=None)

