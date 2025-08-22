
from typing import Optional
from datetime import datetime
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse
from navigation_server.webapp.apps.maps.models import Map


class MapSerializer(BaseModel):
    id: int
    name: str
    description: str
    image: Optional[str]
    resolution: float
    origin_x: float
    origin_y: float
    created_at: datetime
    updated_at: datetime

    def __init__(self, map: Map):
        super().__init__(
            id=map.id,
            name=map.name,
            description=map.description,
            image=map.to_base64(),
            resolution=map.resolution,
            origin_x=map.origin_x,
            origin_y=map.origin_y,
            created_at=map.created_at,
            updated_at=map.updated_at,
        )


class MapSimplestSerializer(BaseModel):
    id: int
    name: str

    def __init__(self, map: Map):
        super().__init__(id=map.id, name=map.name)


class MapForListSerializer(BaseModel):
    id: int
    name: str
    description: str
    created_at: datetime
    updated_at: datetime

    def __init__(self, map: Map):
        super().__init__(
            id=map.id,
            name=map.name,
            description=map.description,
            created_at=map.created_at,
            updated_at=map.updated_at,
        )


class MapListResponseSerializer(DataResponse):
    data: list[MapForListSerializer]


class MapDetailResponseSerializer(DataResponse):
    data: MapSerializer


class MapOnlySerializer(BaseModel):
    id: int
    name: str
    description: str
    resolution: float
    origin_x: float
    origin_y: float
    created_at: datetime
    updated_at: datetime

    def __init__(self, map: Map):
        super().__init__(
            id=map.id,
            name=map.name,
            description=map.description,
            resolution=map.resolution,
            origin_x=map.origin_x,
            origin_y=map.origin_y,
            created_at=map.created_at,
            updated_at=map.updated_at,
        )


class MapOnlyCurrentSerializer(MapOnlySerializer):
    image: str = ""

    def __init__(self, map: Map):
        super().__init__(map)
        self.image = map.to_base64()


class MapOnlyCurrentResponseSerializer(DataResponse):
    data: MapOnlyCurrentSerializer

