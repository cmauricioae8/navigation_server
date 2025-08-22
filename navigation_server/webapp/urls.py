
from fastapi import APIRouter

from navigation_server.webapp.apps.home.urls import router as home_router
from navigation_server.webapp.apps.users.urls import router as user_routers
from navigation_server.webapp.apps.maps.urls import router as map_router
from navigation_server.webapp.apps.waypoints.urls import router as waypoints_router
from navigation_server.webapp.apps.paths.urls import router as paths_router
from navigation_server.webapp.apps.navigation.urls import router as navigation_router
from navigation_server.webapp.apps.ros2_app.urls import router as ros2_router


router = APIRouter()

router.include_router(home_router, prefix="", include_in_schema=False)
router.include_router(user_routers, prefix="/users", tags=["Usuarios"], include_in_schema=False)
router.include_router(map_router, prefix="/maps")
router.include_router(waypoints_router, prefix="/waypoints")
router.include_router(paths_router, prefix="/paths")
router.include_router(navigation_router, prefix="/navigation")
router.include_router(ros2_router, prefix="/ros", tags=["ROS2"])

