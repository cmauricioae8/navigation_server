
from typing import Union
from fastapi import APIRouter, status

from navigation_server.webapp.apps.base.serializers import SimpleResponse
from navigation_server.webapp.apps.paths.cruds.path_cruds import (
    path_crud,
    validate_name_exists,
)
from navigation_server.webapp.apps.paths.forms.path_forms import PathCreateForm
from navigation_server.webapp.apps.paths.models import Path
from navigation_server.webapp.apps.paths.serializers.path_serializers import (
    PathSerializer,
    PathListResponseSerializer,
    PathDetailResponseSerializer,
)
from navigation_server.webapp.apps.base.serializers import ErrorResponse
from navigation_server.webapp.apps.base.errors import ERRORS

router = APIRouter()


@router.get(
    "/", response_model=PathListResponseSerializer, status_code=status.HTTP_200_OK
)
def list_paths():
    """
    Lista de trayectorias

    Este endpoint retorna una lista de trayectorias registradas en formato JSON
    """
    paths = path_crud.list()
    data = [PathSerializer(path) for path in paths]
    return PathListResponseSerializer(status="OK", message="Path list", data=data)


@router.get(
    "/{path_id}/",
    response_model=PathDetailResponseSerializer,
    status_code=status.HTTP_200_OK,
)
def get_path(path_id: int):
    """
    Detalle de la trayectoria

    Este endpoint retorna el detalle de una trayectoria en formato JSON
    """
    path = path_crud.get_or_404(path_id)
    data = PathSerializer(path)
    return PathDetailResponseSerializer(status="OK", message="Path detail", data=data)


@router.get(
    "/by_map/{map_id}/",
    response_model=PathListResponseSerializer,
    status_code=status.HTTP_200_OK,
)
def get_paths_by_map(map_id: int):
    """
    Consultar las trayectorias por mapa

    Este endpoint retorna una lista de trayectorias registrados en formato JSON
    """
    paths = path_crud.get_by_field("map_id", map_id, allows_multiple=True)
    data = [PathSerializer(path) for path in paths]
    return PathListResponseSerializer(
        status="OK", message="Path list by map", data=data
    )


@router.post(
    "/",
    response_model=Union[PathDetailResponseSerializer, ErrorResponse],
    status_code=status.HTTP_201_CREATED,
)
def create_path(path_form: PathCreateForm):
    """
    Crear trayectoria

    Este endpoint permite crear una trayectoria en formato JSON
    """

    path = Path(
        name=path_form.name,
        map_id=path_form.map,
        description=path_form.description,
        stop_waypoints_json=path_form.stop_waypoints_to_dict(),
    )

    # check if path already exists
    if validate_name_exists(path_form.name, path_form.map):
        return ErrorResponse(
            status="FAIL", message="Path already exists", error=ERRORS.NAME_UNIQUE
        )

    path = path_crud.create(path)
    data = PathSerializer(path)
    return PathDetailResponseSerializer(status="OK", message="Path created", data=data)


@router.delete(
    "/{path_id}/", response_model=SimpleResponse, status_code=status.HTTP_200_OK
)
def delete_path(path_id: int):
    """
    Eliminar trayectoria

    Este endpoint permite eliminar una trayectoria
    """
    path_crud.delete(path_id)
    return SimpleResponse(status="OK", message="Path deleted")
