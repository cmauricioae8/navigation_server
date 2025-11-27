
from fastapi import APIRouter

from navigation_server.webapp.apps.params.routers import params_routers

router = APIRouter()

router.include_router(params_routers.router)
