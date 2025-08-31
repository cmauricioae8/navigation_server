
from sqlmodel import Field, Column
from sqlalchemy import JSON

from navigation_server.webapp.apps.base.models import BaseModel
from navigation_server.webapp.apps.waypoints.models import Waypoint


class StopWaypoint:
    def __init__(
        self,
        waypoint: Waypoint,
        attempts: int = 0,  # 0: infinitely, 1: only one, n: n times
        time_attempt: int = 0,  # 0: infinitely, n: n seconds
        stop_time: float = 0.0,  # 0: no stop, n: n seconds
    ):
        self.waypoint = waypoint
        self.attempts = attempts
        self.time_attempt = time_attempt
        self.stop_time = stop_time

    def to_dict(self):
        return self.__dict__.copy()

    def __str__(self):
        return "{} x {:2.2f}s".format(
            self.waypoint, 0.0 if self.stop_time is None else self.stop_time
        )


class Path(BaseModel, table=True):
    name: str = Field(max_length=50)
    map_id: int = Field(foreign_key="map.id")
    stop_waypoints_json: list = Field(default=[], sa_column=Column(JSON))

    def __str__(self):
        return "{} - {}".format(self.id, self.name)

    def stop_waypoints(self) -> list[StopWaypoint]:
        return [StopWaypoint(**sw) for sw in self.stop_waypoints_json]
