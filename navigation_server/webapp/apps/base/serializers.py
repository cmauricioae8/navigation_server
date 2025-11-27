
import logging
from datetime import datetime
from pydantic import BaseModel, Field

from navigation_server.utils import EnumWithDescription

logger = logging.getLogger(__name__)


class BaseSerializer(BaseModel):
    def dump_to_logger(self, indent=0):
        indent_str = "  " * indent
        for k, v in self.__dict__.items():
            if isinstance(v, BaseSerializer):
                logger.info(f"{indent_str}{k}:")
                v.dump_to_logger(indent + 1)
            elif isinstance(v, EnumWithDescription):
                logger.info(f"{indent_str}{k}: \t{v.value}")
            else:
                logger.info(f"{indent_str}{k}: \t{v}")

    def set_from_dict(self, data: dict):
        # remove None values
        data = {k: v for k, v in data.items() if v is not None}
        for k, v in data.items():
            # remove None values from nested dictionaries
            if isinstance(v, dict):
                data[k] = {k: v for k, v in v.items() if v is not None}
            # remove None values from nested lists
            elif isinstance(v, list):
                data[k] = [i for i in v if i is not None]

        for k, v in self.__dict__.items():
            # skip if the attribute is not in the data
            if k not in data:
                continue
            if isinstance(v, BaseSerializer):
                v.set_from_dict(data[k])
            elif isinstance(v, EnumWithDescription):
                try:
                    setattr(self, k, v.__class__(data[k]))
                except ValueError as e:
                    logger.warning(f"Error setting {k}: {e}")
            elif k in data:
                setattr(self, k, data[k])

    def get_dict(self, object) -> dict:
        data_out = {}
        for k, v in object.__dict__.items():
            if isinstance(v, BaseModel):
                data_out[k] = self.get_dict(v)
            elif isinstance(v, EnumWithDescription):
                data_out[k] = v.value
            elif isinstance(v, datetime):
                data_out[k] = v.strftime("%Y-%m-%d %H:%M:%S")
            else:
                data_out[k] = v
        return data_out

    def to_dict(self) -> dict:
        return self.get_dict(self)


class SimpleResponse(BaseModel):
    status: str = Field(examples=["OK", "FAIL"])
    message: str = Field(examples=["Operation successful", "Operation failed"])

class SimpleResponseList(BaseModel):
    status: str = Field(examples=["OK", "FAIL"])
    message: list = Field(examples=["string 1", "String 2"])

class DataResponse(BaseModel):
    status: str = Field(examples=["OK", "FAIL"])
    message: str = Field(examples=["Operation successful", "Operation failed"])
    data: dict = Field(examples=[{"key": "value"}])


class ErrorResponse(BaseModel):
    status: str = Field(examples=["FAIL"])
    message: str = Field(examples=["Operation failed"])
    error: str = Field(examples=["Error message"])


class DataWithPaginationLegacy(BaseModel):
    count: int = Field(examples=[15])
    next: str | None = Field(
        default=None, examples=["http://0.0.0.0:9009/robot/notification/?page=3"]
    )
    previous: str | None = Field(
        default=None, examples=["http://0.0.0.0:9009/robot/notification/?page=1"]
    )
    results: list = Field(examples=[])


class DataWithPagination(BaseModel):
    page: int = Field(examples=[1])
    page_count: int = Field(examples=[10])
    pages: list[str] = Field(examples=[["1", "2", "...", "10"]])
    last_page: int = Field(examples=[10])
    prev_page: int | None = Field(examples=[None])
    next_page: int | None = Field(examples=[2])
    records_count: int = Field(examples=[100])
    records: list = Field(examples=[[]])

