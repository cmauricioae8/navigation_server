
from fastapi import FastAPI, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.requests import Request
from fastapi.openapi.docs import get_swagger_ui_html, get_redoc_html
from fastapi.openapi.utils import get_openapi
from sqlmodel import Session, select

# import socketio

# Import the models to create the tables, don't remove this import even if it's not used
from .apps.users.models import User  # noqa: F401
# from .apps.maps.models import Map # noqa: F401
# from .apps.paths.models import Path  # noqa: F401
# from .apps.waypoints.models import Waypoint # noqa: F401

from .database import engine
from .dependencies import NotAuthenticatedException , static_dir, media_dir
from .urls import router
from .apps.home.routers.login_router import login_manager
# from .ws_no_prefix import NoPrefixNamespace
# from .socket_io import sio


app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

########################################################################################
# swagger docs

@app.get("/docs", include_in_schema=False)
async def get_docs(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return get_swagger_ui_html(
        openapi_url="/openapi.json",
        title="Documentación Swagger",
        swagger_favicon_url="/favicon.ico",
    )

@app.get("/redoc", include_in_schema=False)
async def get_redoc(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return get_redoc_html(
        openapi_url="/openapi.json",
        title="Documentación ReDoc",
        redoc_favicon_url="/favicon.ico",
    )

@app.get("/openapi.json", include_in_schema=False)
async def openapi(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return get_openapi(
        title="Navigation API Server",
        description=("Bridge between ROS 2 ecosystem and the REST API"),
        version="1.0.0",
        routes=app.routes,
    )

"""
sio.register_namespace(NoPrefixNamespace("/"))
sio_asgi_app = socketio.ASGIApp(socketio_server=sio, other_asgi_app=app)
"""

login_manager.useRequest(app)


@login_manager.user_loader()
def get_user(username: str):
    with Session(engine) as session:
        user = session.exec(select(User).where(User.username == username)).first()
        return user


# mount static files
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# mount media files
app.mount("/media", StaticFiles(directory=media_dir), name="media")

# include routers
app.include_router(router)

# app.add_route("/socket.io/", route=sio_asgi_app, methods=["GET", "POST"])
# app.add_websocket_route("/socket.io/", sio_asgi_app)

########################################################################################

@app.exception_handler(NotAuthenticatedException)
def auth_exception_handler(request: Request, exc: NotAuthenticatedException):
    return RedirectResponse("/login", status_code=status.HTTP_303_SEE_OTHER)

