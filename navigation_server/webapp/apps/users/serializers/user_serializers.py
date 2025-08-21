
from datetime import datetime
from pydantic import BaseModel

from navigation_server.webapp.apps.base.serializers import DataResponse, DataWithPagination
from navigation_server.webapp.apps.users.models import User


class UserSerializer(BaseModel):
    id: int
    username: str
    is_active: bool
    is_admin: bool
    created_at: datetime
    updated_at: datetime

class UserResponseSerializer(DataResponse):
    data: UserSerializer = None

    def set_data(self, user: User):
        self.data = UserSerializer(
            id=user.id,
            username=user.username,
            is_active=user.is_active,
            is_admin=user.is_admin,
            created_at=user.created_at,
            updated_at=user.updated_at,
        )

class UserListSerializer(DataWithPagination):
    records: list[UserSerializer]


