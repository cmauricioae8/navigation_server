
from fastapi import APIRouter

from navigation_server.webapp.apps.debug.routers import pm2_routers

router = APIRouter()

router.include_router(pm2_routers.router)
