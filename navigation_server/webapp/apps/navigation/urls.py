
from fastapi import APIRouter

from navigation_server.webapp.apps.navigation.routers import navigation_routers

router = APIRouter()

router.include_router(navigation_routers.router, prefix="/navigation", tags=["Navigation"])
