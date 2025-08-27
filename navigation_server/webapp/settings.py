
import os
import logging
import json
from navigation_server.utils import EnumWithDescription as Enum

logger = logging.getLogger(__name__)

APP_DATA_DIR = os.path.join(os.path.expanduser("~"), ".navigation_server")


class OperationMode(Enum):
    STOP = "stop", "Detenido"
    STATIC = "static", "Estático"
    TELEOPERATION = "teleoperation", "Teleoperación"
    MAPPING = "mapping", "Mapeo"
    WAYPOINTS = "waypoints", "Punto de ruta"
    NAVIGATION = "navigation", "Navegación"


class Process:
    def __init__(
        self,
        ENVIRON_VARS: dict = {},
        COMMANDS: list[str] = [],
        ERROR_CODES: list[str] = [],
    ):
        # The environment variables that the below commands need to run,
        # in the format "ENV_VAR_NAME": "ENV_VAR_VALUE"
        self.ENVIRON_VARS: dict = ENVIRON_VARS
        # The commands to execute to start the process, each command in a new string
        # separated by comma
        self.COMMANDS: list[str] = COMMANDS
        # The error codes that block the process, each error code in a new string
        # separated by comma
        self.ERROR_CODES: list[str] = ERROR_CODES

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

class Map:
    def __init__(
        self,
        WITH_AXIS: bool = True,
        # COLORS: MapColors = MapColors(),
        SAVE_MAP_PROCESS: Process = Process(
            COMMANDS=[
                # commands to execute to save the map, mandatory include the {MAP_FILE}
                # variable.
                "ros2 run nav2_map_server map_saver_cli -f {MAP_FILE}"
            ]
        ),
        # EMIT_EVENT_RATE: int = 10,
    ):
        self.WITH_AXIS: bool = WITH_AXIS  # if true, the map will be published with axis
        # self.COLORS: MapColors = COLORS
        self.SAVE_MAP_PROCESS: Process = SAVE_MAP_PROCESS
        # self.EMIT_EVENT_RATE: int = EMIT_EVENT_RATE  # Hz

    @classmethod
    def from_dict(cls, data: dict):
        # data["COLORS"] = (
        #     MapColors.from_dict(data["COLORS"]) if "COLORS" in data else MapColors()
        # )
        data["SAVE_MAP_PROCESS"] = (
            Process.from_dict(data["SAVE_MAP_PROCESS"])
            if "SAVE_MAP_PROCESS" in data
            else Process()
        )
        return cls(**data)

    def to_dict(self):
        data = self.__dict__.copy()
        # data["COLORS"] = self.COLORS.to_dict()
        data["SAVE_MAP_PROCESS"] = self.SAVE_MAP_PROCESS.to_dict()
        return data

class ModeManager:
    def __init__(
        self,
        MODE_TO_LOAD_ON_STARTUP: OperationMode = None,
        PROCESS_FOR_STATIC: Process = Process(),
        PROCESS_FOR_TELEOPERATION: Process = Process(),
        PROCESS_FOR_MAPPING: Process = Process(
            COMMANDS=["ros2 launch robot_bringup slam_mapping_launch.py"]
        ),
        PROCESS_FOR_WAYPOINTS: Process = Process(
            COMMANDS=[
                (
                    "ros2 launch robot_bringup navigation_mode_launch.py"
                    " map:={MAP_FILE}"
                    " open_rviz:=False"
                )
            ]
        ),
        PROCESS_FOR_NAVIGATION: Process = Process(
            COMMANDS=[
                (
                    "ros2 launch robot_bringup navigation_mode_launch.py"
                    " map:={MAP_FILE}"
                    " open_rviz:=False"
                )
            ]
        ),
    ):
        self.MODE_TO_LOAD_ON_STARTUP: OperationMode = (
            MODE_TO_LOAD_ON_STARTUP  # mapping, manual
        )
        self.PROCESS_FOR_STATIC: Process = PROCESS_FOR_STATIC
        self.PROCESS_FOR_TELEOPERATION: Process = PROCESS_FOR_TELEOPERATION
        self.PROCESS_FOR_MAPPING: Process = PROCESS_FOR_MAPPING
        self.PROCESS_FOR_WAYPOINTS: Process = PROCESS_FOR_WAYPOINTS
        self.PROCESS_FOR_NAVIGATION: Process = PROCESS_FOR_NAVIGATION

    @classmethod
    def from_dict(cls, data: dict):
        data["PROCESS_FOR_STATIC"] = (
            Process.from_dict(data["PROCESS_FOR_STATIC"])
            if "PROCESS_FOR_STATIC" in data
            else Process()
        )
        data["PROCESS_FOR_TELEOPERATION"] = (
            Process.from_dict(data["PROCESS_FOR_TELEOPERATION"])
            if "PROCESS_FOR_TELEOPERATION" in data
            else Process()
        )
        data["PROCESS_FOR_MAPPING"] = (
            Process.from_dict(data["PROCESS_FOR_MAPPING"])
            if "PROCESS_FOR_MAPPING" in data
            else Process()
        )
        data["PROCESS_FOR_WAYPOINTS"] = (
            Process.from_dict(data["PROCESS_FOR_WAYPOINTS"])
            if "PROCESS_FOR_WAYPOINTS" in data
            else Process()
        )
        data["PROCESS_FOR_NAVIGATION"] = (
            Process.from_dict(data["PROCESS_FOR_NAVIGATION"])
            if "PROCESS_FOR_NAVIGATION" in data
            else Process()
        )
        return cls(**data)

    def to_dict(self):
        data = self.__dict__.copy()
        data["PROCESS_FOR_STATIC"] = self.PROCESS_FOR_STATIC.to_dict()
        data["PROCESS_FOR_TELEOPERATION"] = self.PROCESS_FOR_TELEOPERATION.to_dict()
        data["PROCESS_FOR_MAPPING"] = self.PROCESS_FOR_MAPPING.to_dict()
        data["PROCESS_FOR_WAYPOINTS"] = self.PROCESS_FOR_WAYPOINTS.to_dict()
        data["PROCESS_FOR_NAVIGATION"] = self.PROCESS_FOR_NAVIGATION.to_dict()
        return data

    def get_process(self, mode: OperationMode) -> Process:
        if mode == OperationMode.STATIC:
            return self.PROCESS_FOR_STATIC
        elif mode == OperationMode.TELEOPERATION:
            return self.PROCESS_FOR_TELEOPERATION
        elif mode == OperationMode.MAPPING:
            return self.PROCESS_FOR_MAPPING
        elif mode == OperationMode.WAYPOINTS:
            return self.PROCESS_FOR_WAYPOINTS
        elif mode == OperationMode.NAVIGATION:
            return self.PROCESS_FOR_NAVIGATION
        else:
            return Process()

    def get_match_errors(
        self, mode: OperationMode, current_errors: list[str]
    ) -> list[str]:
        process = self.get_process(mode)
        return [error for error in process.ERROR_CODES if error in current_errors]


class Settings:
    def __init__(self):
        
        # mode manager, for modes: static, teleoperation, mapping, waypoints, navigation
        self.POSE_TO_SET: str = "last_mode"  # "last_mode", "home"
        self.MODE_MANAGER: ModeManager = ModeManager()
        self.MAP: Map = Map()
        self.KEEPOUT_ZONES_PROCESS: Process = Process(
            COMMANDS=[
                (
                    "ros2 launch robot_bringup keepout_zones_launch.py"
                    " keepout_mask:={MAP_FILE}"
                )
            ]
        )
        self.SPEED_LIMITS_PROCESS: Process = Process(
            COMMANDS=[
                "ros2 launch robot_bringup speed_limit_launch.py"
                " speed_mask:={MAP_FILE}"
            ]
        )
        # self.NAVIGATION_MANAGER: NavigationManager = NavigationManager()

        self.load()

    def from_dict(self, data: dict):
        self.MODE_MANAGER = (
            ModeManager.from_dict(data["MODE_MANAGER"])
            if "MODE_MANAGER" in data
            else ModeManager()
        )
        self.MAP = Map.from_dict(data["MAP"]) if "MAP" in data else Map()

        self.KEEPOUT_ZONES_PROCESS = (
            Process.from_dict(data["KEEPOUT_ZONES_PROCESS"])
            if "KEEPOUT_ZONES_PROCESS" in data
            else Process()
        )
        self.SPEED_LIMITS_PROCESS = (
            Process.from_dict(data["SPEED_LIMITS_PROCESS"])
            if "SPEED_LIMITS_PROCESS" in data
            else Process()
        )
        # self.NAVIGATION_MANAGER = (
        #     NavigationManager.from_dict(data["NAVIGATION_MANAGER"])
        #     if "NAVIGATION_MANAGER" in data
        #     else NavigationManager()
        # )

    def to_dict(self):
        data = self.__dict__.copy()
        data["MODE_MANAGER"] = self.MODE_MANAGER.to_dict()
        data["MAP"] = self.MAP.to_dict()
        data["KEEPOUT_ZONES_PROCESS"] = self.KEEPOUT_ZONES_PROCESS.to_dict()
        data["SPEED_LIMITS_PROCESS"] = self.SPEED_LIMITS_PROCESS.to_dict()
        # data["NAVIGATION_MANAGER"] = self.NAVIGATION_MANAGER.to_dict()
        return data

    def save(self):
        # create app data dir if not exists
        if not os.path.exists(APP_DATA_DIR):
            os.makedirs(APP_DATA_DIR)
        # save the settings to the file
        with open(APP_DATA_DIR + "/settings.json", "w") as file:
            file.write(json.dumps(self.to_dict(), indent=4))

    def load(self):
        # check if the file exists
        if os.path.exists(APP_DATA_DIR + "/settings.json"):
            # load the settings from the file
            with open(APP_DATA_DIR + "/settings.json", "r") as file:
                data = file.read()
                data = json.loads(data)
                self.from_dict(data)
        else:
            # save the settings to the file
            print("No settings file found, creating a new one.")
            self.save()


settings = Settings()

if __name__ == "__main__":
    settings.save()
    print(settings.to_dict())