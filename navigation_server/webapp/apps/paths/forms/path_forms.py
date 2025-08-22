from pydantic import BaseModel, Field


class StopWaypointForm(BaseModel):
    waypoint: int = Field(
        description="Punto de parada",
        examples=[1, 2],
    )
    attempts: int = Field(
        description="Número de intentos",
        examples=[1, 2],
    )
    time_attempt: int = Field(
        description="Tiempo de intento",
        examples=[1, 2],
    )
    stop_time: float = Field(
        description="Tiempo de parada",
        examples=[1.0, 2.0],
    )


class PathCreateForm(BaseModel):
    name: str = Field(
        description="Nombre de la trayectoria",
        max_length=50,
        unique=True,
        examples=["trayectoria 1", "trayectoria 2"],
    )
    map: int = Field(
        description="Mapa al que pertenece el trayectoria",
        examples=[1, 2],
    )
    stop_waypoints: list[StopWaypointForm] = Field(
        description="Puntos de parada de la trayectoria",
        examples=[
            [
                {
                    "waypoint": 1,
                    "attempts": 3,
                    "time_attempt": 10,
                    "stop_time": 1.0,
                },
                {
                    "waypoint": 2,
                    "attempts": 2,
                    "time_attempt": 10,
                    "stop_time": 2.0,
                },
            ],
        ],
    )

    def stop_waypoints_to_dict(self):
        stop_waypoints = []
        for stop_waypoint in self.stop_waypoints:
            stop_waypoints.append(
                {
                    "waypoint": stop_waypoint.waypoint,
                    "attempts": stop_waypoint.attempts,
                    "time_attempt": stop_waypoint.time_attempt,
                    "stop_time": stop_waypoint.stop_time,
                }
            )
        return stop_waypoints

