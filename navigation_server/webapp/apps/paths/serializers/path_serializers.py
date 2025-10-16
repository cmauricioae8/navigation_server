
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse
from navigation_server.webapp.apps.maps.serializers.map_serializers import MapSimplestSerializer
from navigation_server.webapp.apps.maps.cruds.map_cruds import map_crud_manager
from navigation_server.webapp.apps.paths.models import Path, StopWaypoint
from navigation_server.webapp.apps.waypoints.serializers.waypoint_serializers import WaypointSerializer
from navigation_server.webapp.apps.waypoints.cruds.waypoint_cruds import waypoint_crud


class StopWaypointSerializer(BaseModel):
    waypoint: WaypointSerializer
    attempts: int
    time_attempt: int
    stop_time: float

    def __init__(self, stop_waypoint: StopWaypoint):
        # Get child objects from database
        if isinstance(stop_waypoint.waypoint, int):
            stop_waypoint.waypoint = waypoint_crud.get(stop_waypoint.waypoint)
        super().__init__(
            waypoint=WaypointSerializer(stop_waypoint.waypoint) if stop_waypoint.waypoint else None,
            attempts=stop_waypoint.attempts,
            time_attempt=stop_waypoint.time_attempt,
            stop_time=stop_waypoint.stop_time,
        )


class PathSerializer(BaseModel):
    id: int
    name: str
    description: str
    map: MapSimplestSerializer
    stop_waypoints: list[StopWaypointSerializer]

    def __init__(self, path: Path):
        # Get child objects from database
        map = map_crud_manager.get(path.map_id)
        stop_waypoints = []
        for stop_waypoint in path.stop_waypoints():
            stop_waypoints.append(StopWaypointSerializer(stop_waypoint))
        super().__init__(
            id=path.id,
            name=path.name,
            description=path.description,
            map=MapSimplestSerializer(map) if map else None,
            stop_waypoints=stop_waypoints,
        )


class PathListResponseSerializer(DataResponse):
    data: list[PathSerializer]


class PathDetailResponseSerializer(DataResponse):
    data: PathSerializer


class PathSimplestSerializer(BaseModel):
    id: int | None
    name: str
    description: str | None = None

    def __init__(self, path: Path):
        super().__init__(id=path.id, name=path.name, description=path.description)
