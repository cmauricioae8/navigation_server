from pydantic import BaseModel, Field


class StopWaypointForm(BaseModel):
    waypoint: int = Field(
        description="Punto de parada",
        examples=[1, 2],
    )
    attempts: int = Field(
        description="Número de intentos (0 para obligatorio)",
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
    description: str = Field(
        max_length=250,
        examples=["trayectoria de prueba"],
    )
    map: int = Field(
        description="Mapa al que pertenece el trayectoria",
        examples=[1, 2],
    )
    stop_waypoints: list[StopWaypointForm] = Field(
        description="Puntos de parada de la trayectoria (attempts=0, obligatorio)",
        examples=[
            [
                {
                    "waypoint": 1,
                    "attempts": 0,
                    "stop_time": 1.0,
                },
                {
                    "waypoint": 2,
                    "attempts": 3,
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
                    "stop_time": stop_waypoint.stop_time,
                }
            )
        return stop_waypoints

