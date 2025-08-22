
from sqlmodel_crud_manager.crud import CRUDManager
from sqlmodel import Session, select

from navigation_server.webapp.database import engine
from navigation_server.webapp.apps.waypoints.models import Waypoint

waypoint_crud = CRUDManager(Waypoint, engine)


def validate_name_exists(name: str, map_id: int) -> bool:
    with Session(engine) as session:
        statement = select(Waypoint).where(
            Waypoint.name == name, Waypoint.map_id == map_id
        )
        waypoint = session.exec(statement).first()
        if waypoint:
            return True
        return False
