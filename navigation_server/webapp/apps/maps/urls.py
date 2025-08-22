
from fastapi import APIRouter

from navigation_server.webapp.apps.maps.routers import map_routers


router = APIRouter()

router.include_router(map_routers.router, prefix="/maps", tags=["Maps"])

