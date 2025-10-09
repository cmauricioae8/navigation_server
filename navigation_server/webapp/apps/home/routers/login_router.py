
import logging
from datetime import timedelta

from fastapi import APIRouter, Depends, status
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.requests import Request
from fastapi.security import OAuth2PasswordRequestForm
from fastapi_login import LoginManager

from navigation_server.webapp.dependencies import NotAuthenticatedException, templates
from navigation_server.webapp.apps.users.cruds.user_cruds import user_crud
from navigation_server.webapp.apps.users.models import User, UserLogin

logger = logging.getLogger(__name__)

router = APIRouter()


login_manager = LoginManager(
    "6558c21a0068b9a84411f5037dcb76a422c0e73aad5a96bba92ded0d9bd5583f",
    "/login_manager",
    use_cookie=True,
    use_header=False,
    default_expiry=timedelta(hours=12),
    custom_exception=NotAuthenticatedException,
)


@router.get("/login_manager", response_class=HTMLResponse)
async def get_login(
    request: Request, message_color: str = None, message_title: str = None
):
    return templates.TemplateResponse(
        "home/login.html",
        {
            "request": request,
            "message_color": message_color,
            "message_title": message_title,
        },
    )


@router.post("/login_manager")
async def post_login(data: OAuth2PasswordRequestForm = Depends()):
    username = data.username
    plain_password = data.password

    user: User = user_crud.get_by_field("username", username)

    if user is None:
        return RedirectResponse(
            "/login_manager?message_color=danger&message_title=Usuario no registrado",
            status_code=status.HTTP_303_SEE_OTHER,
        )
    elif not user.verify_password(plain_password):
        return RedirectResponse(
            "/login_manager?message_color=warning&message_title=Contraseña incorrecta",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    token = login_manager.create_access_token(data={"sub": username})
    logger.info(f"User {username} logged in, with token {token}")
    response = RedirectResponse("/", status_code=status.HTTP_303_SEE_OTHER)
    login_manager.set_cookie(response, token)
    return response  # return {'access_token': token}



@router.post("/login")
async def post_login(data: UserLogin):
    username = data.Nombre
    plain_password = data.Password

    user: User = user_crud.get_by_field("username", username)

    if user is None:
        return {"Status": False, "Description": "Usuario incorrecto."}
    elif not user.verify_password(plain_password):
        return {"Status": False, "Description": "Contraseña incorrecta."}

    token = login_manager.create_access_token(data={"sub": username})
    logger.warning(f"User {username} logged in, with token {token}")
    return {'Status': True, "token": token}


@router.get("/logout_manager")
async def get_logout(request: Request):
    response = RedirectResponse("/login_manager", status_code=status.HTTP_303_SEE_OTHER)
    response.delete_cookie(key="access-token")
    return response


@router.delete("/logout")
async def get_logout():
    return {"status": True, "descripcion": "Sesion expirada."}