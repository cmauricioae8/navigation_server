
from fastapi import APIRouter

from navigation_server.webapp.apps.waypoints.routers import waypoint_routers


router = APIRouter()

router.include_router(waypoint_routers.router, prefix="/waypoint", tags=["Waypoints"])

