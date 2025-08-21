
import os
import logging
from unipath import Path
from fastapi.templating import Jinja2Templates

from .config import APP_DATA_DIR


logger = logging.getLogger("dependencies")

BASE_DIR = Path(__file__).ancestor(2)
print(f"BASE_DIR: {BASE_DIR}")

static_dir = BASE_DIR.child("webapp").child("static")
templates_dir = BASE_DIR.child("webapp").child("templates")
media_dir = Path(APP_DATA_DIR).child("media")
os.makedirs(media_dir, exist_ok=True) # create dir if not exists

templates = Jinja2Templates(directory=templates_dir)


class NotAuthenticatedException(Exception):
    pass


DATABASE_URL = "sqlite:///" + APP_DATA_DIR + "/database.db"
logger.info("Using sqlite")
logger.info(f"DATABASE_URL: {DATABASE_URL}")
