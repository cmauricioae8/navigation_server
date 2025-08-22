
from sqlmodel_crud_manager.crud import CRUDManager
from sqlmodel import Session, select

from navigation_server.webapp.database import engine
from navigation_server.webapp.apps.paths.models import Path

path_crud = CRUDManager(Path, engine)

def validate_name_exists(name: str, map_id: int) -> bool:
    with Session(engine) as session:
        statement = select(Path).where(
            Path.name == name, Path.map_id == map_id
        )
        waypoint = session.exec(statement).first()
        if waypoint:
            return True
        return False