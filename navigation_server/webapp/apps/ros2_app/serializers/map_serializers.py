
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse


class MapDataSerializer(BaseModel):
    image: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_rad: float


class PointDataSerializer(BaseModel):
    x: float
    y: float


class ScanDataSerializer(BaseModel):
    length: int
    points: list[PointDataSerializer]


class PoseDataSerializer(BaseModel):
    position_x: float
    position_y: float
    orientation: float


class PathPlanDataSerializer(BaseModel):
    length: int
    poses: list[PoseDataSerializer]


class MapFullDataSerializer(BaseModel):
    map_data: MapDataSerializer
    costmap_data: MapDataSerializer | None = None
    scan_data: ScanDataSerializer | None = None
    path_plan_data: PathPlanDataSerializer | None = None


class MapResponseSerializer(DataResponse):
    data: MapFullDataSerializer
