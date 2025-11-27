
from fastapi import APIRouter

from navigation_server.webapp.apps.debug.routers import all_routers

router = APIRouter()

router.include_router(all_routers.router)
