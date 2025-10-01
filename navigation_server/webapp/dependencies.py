
import os
import logging
from unipath import Path
from fastapi.templating import Jinja2Templates
from ament_index_python import get_package_share_directory

from .settings import PACKAGE_NAME, APP_DATA_DIR, DEBUGGING_MODE


logger = logging.getLogger("dependencies")

if DEBUGGING_MODE:
    BASE_DIR = Path(__file__).ancestor(2)
    static_dir = BASE_DIR.child("webapp").child("static")
    templates_dir = BASE_DIR.child("webapp").child("templates")
else:
    BASE_DIR = get_package_share_directory(PACKAGE_NAME)
    static_dir = os.path.join(BASE_DIR, 'navigation_server', 'webapp', 'static')
    templates_dir = os.path.join(BASE_DIR, 'navigation_server', 'webapp', 'templates')
    
print(f"BASE_DIR: {BASE_DIR}")

media_dir = Path(APP_DATA_DIR).child("media")
os.makedirs(media_dir, exist_ok=True) # create dir if not exists

templates = Jinja2Templates(directory=templates_dir)


class NotAuthenticatedException(Exception):
    pass


DATABASE_URL = "sqlite:///" + APP_DATA_DIR + "/database.db"
logger.info("Using sqlite")
logger.info(f"DATABASE_URL: {DATABASE_URL}")
