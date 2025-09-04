
from fastapi import APIRouter

from navigation_server.webapp.apps.home.routers import login_router
from navigation_server.webapp.apps.home.routers import home_router

router = APIRouter()

router.include_router(login_router.router)
router.include_router(home_router.router)
