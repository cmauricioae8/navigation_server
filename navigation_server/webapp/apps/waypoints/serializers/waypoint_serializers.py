
from typing import Optional
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse
from navigation_server.webapp.apps.maps.serializers.map_serializers import MapSimplestSerializer
from navigation_server.webapp.apps.maps.cruds.map_cruds import map_crud_manager
from navigation_server.webapp.apps.waypoints.models import Waypoint


class WaypointSerializer(BaseModel):
    id: int
    name: str
    is_mandatory: bool
    description: Optional[str]
    map: MapSimplestSerializer
    position_x: Optional[float]
    position_y: Optional[float]
    orientation: Optional[float]

    def __init__(self, waypoint: Waypoint):
        # Get child objects from database
        # waypoint_type = waypoint_type_crud_manager.get(waypoint.type)
        map = map_crud_manager.get(waypoint.map_id)
        super().__init__(
            id=waypoint.id,
            name=waypoint.name,
            is_mandatory=waypoint.is_mandatory,
            description=waypoint.description,
            map=MapSimplestSerializer(map) if map else None,
            position_x=round(waypoint.position_x, 2) if waypoint.position_x else 0.0,
            position_y=round(waypoint.position_y, 2) if waypoint.position_y else 0.0,
            orientation=round(waypoint.orientation, 2) if waypoint.orientation else 0.0,
        )


class WaypointListResponseSerializer(DataResponse):
    data: list[WaypointSerializer]


class WaypointDetailResponseSerializer(DataResponse):
    data: WaypointSerializer


class WaypointSimplestSerializer(BaseModel):
    id: int
    name: str

    def __init__(self, waypoint: Waypoint):
        super().__init__(id=waypoint.id, name=waypoint.name)
