from pydantic import BaseModel, Field


class SetPoseRequestForm(BaseModel):
    position_x: float = Field(
        description="Posición en x, en metros",
        examples=[0.0, 1.0, 2.0],
    )
    position_y: float = Field(
        description="Posición en y, en metros",
        examples=[0.0, 1.0, 2.0],
    )
    orientation: float = Field(
        description="Orientación en radianes",
        examples=[0.0, 1.0, 2.0],
    )


class SetPoseToWaypointRequestForm(BaseModel):
    waypoint_id: int = Field(
        description="ID del waypoint",
        examples=[1, 2, 3],
    )
