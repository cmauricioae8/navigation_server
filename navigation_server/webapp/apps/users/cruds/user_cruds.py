
import logging

from sqlmodel_crud_manager.crud import CRUDManager
from sqlmodel import Session, select
from sqlalchemy import func


from navigation_server.webapp.database import engine
from navigation_server.webapp.apps.users.models import User
from navigation_server.webapp.apps.users.serializers.user_serializers import UserSerializer, UserListSerializer

user_crud = CRUDManager(User, engine)

logger = logging.getLogger(__name__)


def get_user(user_id: int) -> User:
    with Session(engine) as session:
        user = session.get(User, user_id)
        return user


def get_users(
    username: str = None,
    is_active: bool = None,
    is_admin: bool = None,
    page: int = 1,
    page_limit: int = 10,
) -> UserListSerializer:
    page_count = 0

    # create statement
    statement = select(User)
    statement_count = select(func.count(User.id))
    if username is not None:
        statement = statement.where(User.username >= username)
        statement_count = statement_count.where(User.username >= username)
    if is_active is not None:
        statement = statement.where(User.is_active == is_active)
        statement_count = statement_count.where(User.is_active == is_active)
    if is_admin is not None:
        statement = statement.where(User.is_admin == is_admin)
        statement_count = statement_count.where(User.is_admin == is_admin)
    statement = statement.order_by(User.id)

    with Session(engine) as session:
        records_count = session.exec(statement_count).first()
        if page_limit < 1:
            page_limit = 1
        if page_limit > 1000:
            page_limit = 1000
        page_count = records_count // page_limit + 1
        if page > page_count:
            page = page_count
        records = session.exec(
            statement.offset((page - 1) * page_limit).limit(page_limit)
        ).all()

        pages = list(range(1, page_count + 1))
        pages = list(map(str, pages))
        if len(pages) > 10:
            if page < 6:
                pages = pages[:10]
                pages.append("...")
            elif page > len(pages) - 5:
                pages = pages[-10:]
                pages.insert(0, "...")
            else:
                pages = pages[page - 5 : page + 5]  # noqa E203
                pages.insert(0, "...")
                pages.append("...")

        records_serialized: list[UserSerializer] = []
        for record in records:
            record_serialized = UserSerializer(
                id=record.id,
                username=record.username,
                is_active=record.is_active,
                is_admin=record.is_admin,
                created_at=record.created_at,
                updated_at=record.updated_at,
                created_by=record.created_by,
                updated_by=record.updated_by,
            )
            records_serialized.append(record_serialized)

        return UserListSerializer(
            page=page,
            page_count=page_count,
            pages=pages,
            last_page=page_count,
            prev_page=page - 1 if page > 1 else None,
            next_page=page + 1 if page < page_count else None,
            records_count=records_count,
            records=records_serialized,
        )
