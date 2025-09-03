
import re
import yaml
from typing import Union
from subprocess import Popen
from os import path, makedirs, environ

from fastapi import APIRouter, status
import cv2 as cv

from navigation_server.webapp.apps.base.serializers import SimpleResponse
from navigation_server.webapp.apps.maps.cruds.map_cruds import (
    map_crud_manager,
    validate_name_exists,
)
from navigation_server.webapp.apps.paths.cruds.path_cruds import path_crud
from navigation_server.webapp.apps.waypoints.cruds.waypoint_cruds import waypoint_crud
from navigation_server.webapp.apps.maps.forms.map_forms import MapCreateForm
from navigation_server.webapp.apps.maps.models import Map
from navigation_server.webapp.apps.paths.models import Path
from navigation_server.webapp.apps.waypoints.models import Waypoint
from navigation_server.webapp.apps.maps.serializers.map_serializers import (
    MapSerializer,
    MapForListSerializer,
    MapListResponseSerializer,
    MapDetailResponseSerializer,
)
from navigation_server.webapp.apps.base.serializers import ErrorResponse
from navigation_server.webapp.apps.base.errors import ERRORS
from navigation_server.webapp.settings import OperationMode, settings, APP_DATA_DIR
from navigation_server.base_node import base_node
from navigation_server.modules.mode_manager import mode_manager

router = APIRouter()


@router.get(
    "/", response_model=MapListResponseSerializer, status_code=status.HTTP_200_OK
)
def list_maps():
    """
    Lista de mapas

    Este endpoint retorna una lista de mapas registrados en formato JSON
    """
    maps = map_crud_manager.list()
    data = [MapForListSerializer(map) for map in maps]
    return MapListResponseSerializer(status="OK", message="Map list", data=data)


@router.get(
    "/{map_id}/",
    response_model=Union[MapDetailResponseSerializer, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def get_map(map_id: int):
    """
    Detalle de mapa

    Este endpoint retorna el detalle de un mapa en formato JSON
    """
    map = Map().get(map_id)
    if not map:
        return ErrorResponse(
            status="FAIL", message="Map not found", error=ERRORS.OBJECT_NOT_FOUND
        )
    data = MapSerializer(map)
    return MapDetailResponseSerializer(status="OK", message="Map detail", data=data)


@router.post(
    "/",
    response_model=Union[MapDetailResponseSerializer, ErrorResponse],
    status_code=status.HTTP_201_CREATED,
)
def create_map(map_form: MapCreateForm):
    """
    Crear mapa

    Este endpoint permite crear un mapa. Se crean automáticamente sus archivos _keepout y _speed
    """
    if mode_manager.mode != OperationMode.MAPPING:
        return ErrorResponse(
            status="FAIL",
            message="No se puede guardar el mapa",
            error=ERRORS.NO_AVAILABLE_IN_MODE,
        )
    if not mode_manager.mode_ready:
        return ErrorResponse(
            status="FAIL",
            message="No se puede guardar el mapa",
            error=ERRORS.MODE_NOT_READY,
        )
    if validate_name_exists(map_form.name):
        return ErrorResponse(
            status="FAIL",
            message="No se puede guardar el mapa",
            error=ERRORS.NAME_UNIQUE,
        )
    map_name = map_form.name.replace(" ", "_")
    map_name = re.sub(r"[^a-zA-Z0-9_]", "", map_name)

    maps_dir = path.join(APP_DATA_DIR, "media/maps")
    if not path.exists(maps_dir):
        makedirs(maps_dir)

    map_file = path.join(APP_DATA_DIR, "media/maps/", map_name)
    # get the map image from the gmapping node
    processes = []
    for command in settings.MAP.SAVE_MAP_PROCESS.COMMANDS:
        base_node.logger.info("Starting process for save map: {}".format(command))

        if "{MAP_FILE}" in command:
            base_node.logger.info("Map file: {}".format(map_file))
            command = command.replace("{MAP_FILE}", map_file)

        processes.append(Popen(command.split()))

    base_node.logger.info("Wait for saving map processes to finish")
    for process in processes:
        process.wait()
    base_node.logger.info("Save map processes finished")

    # create the map object
    map = Map(
        name=map_form.name,
        description=map_form.description,
    )

    # insert the map image
    map.yaml_path = "maps/" + map_name + ".yaml"
    map.pgm_path = "maps/" + map_name + ".pgm"

    # duplicate the map image to create the original image
    map_pgm_original_path = "maps/" + map_name + "_original.pgm"
    img = cv.imread(
        path.join(APP_DATA_DIR, "media/", map.pgm_path), cv.IMREAD_GRAYSCALE
    )
    cv.imwrite(path.join(APP_DATA_DIR, "media/", map_pgm_original_path), img)

    with open(path.join(APP_DATA_DIR, "media/", map.yaml_path), "r") as file:
        yaml_data = yaml.safe_load(file)

    # set the map resolution and origin
    map.resolution = yaml_data["resolution"]
    map.origin_x = yaml_data["origin"][0]
    map.origin_y = yaml_data["origin"][1]

    map.save()
    map.generate_files_with_keepout_zones()
    map.generate_files_with_speed_limits()
    data = MapSerializer(map)
    return MapDetailResponseSerializer(status="OK", message="Map created", data=data)


@router.delete(
    "/{map_id}/",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def delete_map(map_id: int):
    """
    Eliminar mapa

    Este endpoint permite eliminar un mapa, y sus correspondientes archivos _keepout y _speed
    """
    map: Map = Map().get(map_id)
    if map is None:
        return ErrorResponse(
            status="FAIL", message="Map not found", error=ERRORS.OBJECT_NOT_FOUND
        )
    # delete paths, with crud manager
    paths = path_crud.get_by_field("map_id", map_id, allows_multiple=True)
    path: Path
    for path in paths:
        path_crud.delete(path.id)
    # delete waypoints, with crud manager
    waypoints = waypoint_crud.get_by_field("map_id", map_id, allows_multiple=True)
    waypoint: Waypoint
    for waypoint in waypoints:
        waypoint_crud.delete(waypoint.id)

    map.delete()
    return SimpleResponse(status="OK", message="Map deleted")
