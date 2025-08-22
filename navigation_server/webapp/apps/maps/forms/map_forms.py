from pydantic import BaseModel, Field


class MapCreateForm(BaseModel):
    name: str = Field(
        description="Nombre del mapa",
        max_length=255,
        unique=True,
        examples=["mapa1", "mapa2"],
    )
    description: str = Field(
        default="", description="Descripción del mapa", examples=["mapa1", "mapa2"]
    )

