
from fastapi import APIRouter

from navigation_server.webapp.apps.ros2_app.routers import (
    battery_routers,
    map_routers,
    operation_mode_routers,
    set_pose_routers,
)

router = APIRouter()

router.include_router(battery_routers.router)
router.include_router(map_routers.router)
router.include_router(operation_mode_routers.router)
router.include_router(set_pose_routers.router)
