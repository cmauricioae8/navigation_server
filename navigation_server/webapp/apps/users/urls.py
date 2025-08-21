
from fastapi import APIRouter

from navigation_server.webapp.apps.users.routers import user_routers


router = APIRouter()

router.include_router(user_routers.router, prefix="/user")
