from navigation_server.utils import EnumWithDescription as Enum


class Mode(Enum):
    UNIQUE = "unique", "Único"
    EVENT = "event", "Evento"
    STOP = "stop", "Detener"
