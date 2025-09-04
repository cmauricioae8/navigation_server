from pydantic import BaseModel, Field


class PointDataForm(BaseModel):
    x: float = Field(
        description="Coordenada x en metros",
        examples=[1.0, 2.0],
    )
    y: float = Field(
        description="Coordenada y en metros",
        examples=[1.0, 2.0],
    )
