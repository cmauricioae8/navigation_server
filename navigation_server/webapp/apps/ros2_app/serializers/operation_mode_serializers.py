
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse
from navigation_server.webapp.settings import OperationMode


class OperationModeSerializer(BaseModel):
    mode: OperationMode
    ready: bool


class OperationModeResponseSerializer(DataResponse):
    data: OperationModeSerializer
