
from sqlmodel_crud_manager.crud import CRUDManager
from sqlmodel import Session, select

from navigation_server.webapp.database import engine
from navigation_server.webapp.apps.maps.models import Map

map_crud_manager = CRUDManager(Map, engine)


def validate_name_exists(name: str) -> bool:
    with Session(engine) as session:
        statement = select(Map).where(Map.name == name)
        map = session.exec(statement).first()
        if map:
            return True
        return False
