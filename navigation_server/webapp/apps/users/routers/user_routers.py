
import logging
from typing import Union
from fastapi import APIRouter
from fastapi.requests import Request

from navigation_server.webapp.apps.base.serializers import ErrorResponse
from navigation_server.webapp.apps.users.cruds.user_cruds import user_crud
from navigation_server.webapp.apps.users.forms.user_forms import UserCreateForm
from navigation_server.webapp.apps.users.serializers.user_serializers import UserResponseSerializer


logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/", response_model=Union[UserResponseSerializer, ErrorResponse])
def user_create(request: Request, form: UserCreateForm):
    """
    Nuevo usuario

    Crear un nuevo usuario.
    """
    # check if username already exists
    user = user_crud.get_by_field("username", form.username)
    if user is not None:
        return ErrorResponse(
            status="FAIL",
            message="El nombre de usuario ya existe",
            error="El nombre de usuario ya existe",
        )

    # check if password and confirm password match
    if form.password == "" or form.password != form.password_confirm:
        return ErrorResponse(
            status="FAIL",
            message="Las contraseñas no coinciden",
            error="Las contraseñas no coinciden",
        )
    # create user
    user = form.save()

    # return user detail
    response = UserResponseSerializer(
        status="OK",
        message="Usuario creado",
    )
    response.set_data(user)
    return response

