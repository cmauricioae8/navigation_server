
from datetime import datetime
from pydantic import BaseModel, Field

from navigation_server.webapp.apps.users.models import User


class UserCreateForm(BaseModel):
    username: str = Field(
        title="Nombre de usuario",
        description="Nombre de usuario",
        max_length=100,
        examples=["admin"],
    )
    password: str = Field(
        title="Contraseña",
        description="Contraseña",
        max_length=100,
        examples=["admin"],
    )
    password_confirm: str = Field(
        title="Confirmar contraseña",
        description="Confirmar contraseña",
        max_length=100,
        examples=["admin"],
    )
    is_active: bool = Field(
        title="Activo?",
        description="Activo?",
        default=True,
        examples=[True],
    )
    is_admin: bool = Field(
        title="Administrador?",
        description="Administrador?",
        default=False,
        examples=[False],
    )

    def save(self) -> User:
        user = User(
            username=self.username, is_active=self.is_active, is_admin=self.is_admin
        )
        user.set_password(self.password)
        user.created_at = datetime.now()
        user.save()
        return user


