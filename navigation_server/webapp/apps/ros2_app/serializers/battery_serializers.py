
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse


class BatterySerializer(BaseModel):
    power_supply_status: str
    voltage: float
    percentage: int


class BatteryResponseSerializer(DataResponse):
    data: BatterySerializer
