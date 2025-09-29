
import os
import logging
from unipath import Path
from fastapi.templating import Jinja2Templates
from ament_index_python import get_package_share_directory

from .settings import PACKAGE_NAME, APP_DATA_DIR


logger = logging.getLogger("dependencies")

package_share_directory = get_package_share_directory(PACKAGE_NAME)
print(f"SHARE_DIR: {package_share_directory}")


static_dir = os.path.join(package_share_directory, 'navigation_server', 'webapp', 'static')
templates_dir = os.path.join(package_share_directory, 'navigation_server', 'webapp', 'templates')
media_dir = Path(APP_DATA_DIR).child("media")
os.makedirs(media_dir, exist_ok=True) # create dir if not exists

templates = Jinja2Templates(directory=templates_dir)


class NotAuthenticatedException(Exception):
    pass


DATABASE_URL = "sqlite:///" + APP_DATA_DIR + "/database.db"
logger.info("Using sqlite")
logger.info(f"DATABASE_URL: {DATABASE_URL}")
