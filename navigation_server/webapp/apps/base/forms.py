from pydantic import BaseModel, Field

"""
class EndpointForm(BaseModel):
    url: str | None = Field(
        default=None,
        description="URL del endpoint",
        examples=["http://localhost:8000/notify_results"],
    )
    method: str | None = Field(
        default=None,
        description="Método HTTP para enviar los resultados",
        examples=["POST"],
    )
    payload: dict | None = Field(
        default=None,
        description="Payload a enviar al endpoint",
        examples=[{"results": []}],
    )
    headers: dict | None = Field(
        default=None,
        description="Headers a enviar al endpoint",
        examples=[{"Content-Type": "application/json"}],
    )
"""

class PointDataForm(BaseModel):
    x: float = Field(
        description="Coordenada x en metros",
        examples=[1.0, 2.0],
    )
    y: float = Field(
        description="Coordenada y en metros",
        examples=[1.0, 2.0],
    )
