
from os.path import join
import yaml
from typing import Optional
from sqlmodel import Field
import cv2 as cv
from cv2 import imread, imwrite


from navigation_server.webapp.apps.base.models import BaseModel
from navigation_server.utils import (
    image_to_base64_str
)
from navigation_server.webapp.config import APP_DATA_DIR, try_delete_media_file


class IndentDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(IndentDumper, self).increase_indent(flow, False)


class Map(BaseModel, table=True):
    name: str = Field(max_length=255, unique=True)
    description: Optional[str] = Field(default=None)
    pgm_path: Optional[str] = Field(default=None)
    yaml_path: Optional[str] = Field(default=None)
    resolution: float = Field(default=0.05)
    origin_x: float = Field(default=0.0)
    origin_y: float = Field(default=0.0)

    def to_base64(self) -> str:
        img = imread(join(APP_DATA_DIR, "media/", self.pgm_path), cv.IMREAD_GRAYSCALE)
        return image_to_base64_str(img)

    def generate_files_with_keepout_zones(self) -> None:
        print(f"Generating files with keepout zones for {self.name}")
        img = imread(join(APP_DATA_DIR, "media/", self.pgm_path))
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        with open(join(APP_DATA_DIR, "media/", self.yaml_path), "r") as f:
            yaml_data = yaml.safe_load(f)
        print("keepout_zones yaml_data", yaml_data)

        new_pgm_file_path = self.pgm_path.split(".")[0] + "_keepout.pgm"
        new_yaml_file_path = self.yaml_path.split(".")[0] + "_keepout.yaml"

        # replace data in yaml file
        yaml_data["image"] = new_pgm_file_path.split("/")[-1]

        # save the map with keepout zones
        imwrite(join(APP_DATA_DIR, "media/", new_pgm_file_path), img)
        # save the yaml file with keepout zones
        with open(join(APP_DATA_DIR, "media/", new_yaml_file_path), "w") as f:
            yaml.dump(yaml_data, f, Dumper=IndentDumper, sort_keys=False)

        self.save()

    def generate_files_with_speed_limits(self) -> None:
        print("Generating files with speed limits")
        img = imread(join(APP_DATA_DIR, "media/", self.pgm_path))
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        with open(join(APP_DATA_DIR, "media/", self.yaml_path), "r") as f:
            yaml_data = yaml.safe_load(f)

        new_pgm_file_path = self.pgm_path.split(".")[0] + "_speed.pgm"
        new_yaml_file_path = self.yaml_path.split(".")[0] + "_speed.yaml"

        # replace the data in yaml file
        yaml_data["image"] = new_pgm_file_path.split("/")[-1]
        yaml_data["mode"] = "scale"
        yaml_data["occupied_thresh"] = 1.0
        yaml_data["free_thresh"] = 0.0

        # save the map with speed limits
        imwrite(join(APP_DATA_DIR, "media/", new_pgm_file_path), img)
        # save the yaml file with speed limits
        with open(join(APP_DATA_DIR, "media/", new_yaml_file_path), "w") as f:
            yaml.dump(yaml_data, f, Dumper=IndentDumper, sort_keys=False)

        self.save()

    def delete(self) -> None:
        # delete files
        base_path = self.pgm_path.split(".")[0]

        map_files = [
            self.pgm_path, base_path+".yaml", base_path+"_original.pgm", 
            base_path+"_keepout.pgm", base_path+"_keepout.yaml",
            base_path+"_speed.pgm", base_path+"_speed.yaml"
        ]
        for file in map_files:
            try_delete_media_file(file)

        return super().delete()


