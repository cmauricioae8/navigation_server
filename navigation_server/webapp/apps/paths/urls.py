
from fastapi import APIRouter

from navigation_server.webapp.apps.paths.routers import path_routers


router = APIRouter()

router.include_router(path_routers.router, prefix="/path", tags=["Paths"])

