from pydantic import BaseModel, Field
import yaml
from typing import Any
from os import path
from navigation_server.webapp.settings import PACKAGE_NAME, SERVER_NODE_NAME

safe_motion_robot_path = path.join(path.expanduser("~"),".robot_config",PACKAGE_NAME,"safe_motion_robot.yaml")
safe_motion_sensors_path = path.join(path.expanduser("~"),".robot_config",PACKAGE_NAME,"safe_motion_sensors.yaml")
server_node_path = path.join(path.expanduser("~"),".robot_config",PACKAGE_NAME,SERVER_NODE_NAME+".yaml")


class SetRobotParamsForm(BaseModel):
    max_vel_x: float = Field(
        description="Máxima velocidad lineal",
        # default=None, # Required field to avoid finding params that have changed ---
        examples=[0.6],
    )
    min_vel_x: float = Field(
        description="Mínima velocidad lineal (negativa)",
        examples=[-0.2],
    )
    max_vel_theta: float = Field(
        description="Máxima velocidad angular",
        examples=[0.8],
    )
    manual_vel_gain: float = Field(
        description="Factor multiplicador de velocidad",
        examples=[1.5],
    )

    def to_ros_params_dict(self) -> dict[str, Any]:
        """
        Converts the Pydantic model to the specific ROS parameter dictionary structure:
        { "/**": { "ros__parameters": { ... } } }
        """
        # Get the model's attributes as a dictionary
        model_data = self.model_dump() 
        
        # Build the target structure
        ros_params_dict = {
            "/**": {
                "ros__parameters": model_data
            }
        }
        return ros_params_dict

    def save_to_ros_yaml(self) -> None:
        """
        Restructures the model data and saves it as a YAML file.
        """
        # Get the data in the desired ROS format
        data_to_dump = self.to_ros_params_dict()
        
        # Save the dictionary to a YAML file
        try:
            with open(safe_motion_robot_path, 'w') as f:
                # Use yaml.dump to write the dictionary to the file
                yaml.dump(data_to_dump, f, default_flow_style=False)
            print(f"Successfully saved to {safe_motion_robot_path}")
        except Exception as e:
            print(f"An error occurred while writing the file: {e}")


class SetSensorProtectForm(BaseModel):
    use_lidar: bool = Field(
        description="Bloqueo por mediciones del LiDAR (frontal, lateral)",
        # default=None, # Required field to avoid finding params that have changed ---
        examples=[True, False],
    )
    use_voltage: bool = Field(
        description="Bloqueo por bajo nivel de batería",
        examples=[True, False],
    )
    use_sonars: bool = Field(
        description="Bloqueo por mediciones de sensores ultrasónicos",
        examples=[True, False],
    )
    use_imu: bool = Field(
        description="Bloqueo por exceso de inclinación del robot",
        examples=[True, False],
    )

    def to_ros_params_dict(self) -> dict[str, Any]:
        """
        Converts the Pydantic model to the specific ROS parameter dictionary structure:
        { "/**": { "ros__parameters": { ... } } }
        """
        # Get the model's attributes as a dictionary
        model_data = self.model_dump() 
        
        # Build the target structure
        ros_params_dict = {
            "/**": {
                "ros__parameters": model_data
            }
        }
        return ros_params_dict

    def save_to_ros_yaml(self) -> None:
        """
        Restructures the model data and saves it as a YAML file.
        """
        # Get the data in the desired ROS format
        data_to_dump = self.to_ros_params_dict()
        
        # Save the dictionary to a YAML file
        try:
            with open(safe_motion_sensors_path, 'w') as f:
                # Use yaml.dump to write the dictionary to the file
                yaml.dump(data_to_dump, f, default_flow_style=False)
            print(f"Successfully saved to {safe_motion_sensors_path}")
        except Exception as e:
            print(f"An error occurred while writing the file: {e}")


class SetNavParamsForm(BaseModel):
    nav_distance_tol: float = Field(
        description="Tolerancia permitida para un punto meta (m)",
        # default=None, # Required field to avoid finding params that have changed ---
        examples=[0.35],
    )
    nav_orientation_tol: float = Field(
        description="Tolerancia permitida para un punto meta (rad)",
        examples=[0.35],
    )
    action_in_paused: bool = Field(
        description="Activación de una acción en navagación pausada",
        examples=[True, False],
    )

    def to_ros_params_dict(self) -> dict[str, Any]:
        """
        Converts the Pydantic model to the specific ROS parameter dictionary structure:
        { "/**": { "ros__parameters": { ... } } }
        """
        # Get the model's attributes as a dictionary
        model_data = self.model_dump() 
        
        # Build the target structure
        ros_params_dict = {
            "/**": {
                "ros__parameters": model_data
            }
        }
        return ros_params_dict

    def save_to_ros_yaml(self) -> None:
        """
        Restructures the model data and saves it as a YAML file.
        """
        # Get the data in the desired ROS format
        data_to_dump = self.to_ros_params_dict()
        
        # Save the dictionary to a YAML file
        try:
            with open(server_node_path, 'w') as f:
                # Use yaml.dump to write the dictionary to the file
                yaml.dump(data_to_dump, f, default_flow_style=False)
            print(f"Successfully saved to {server_node_path}")
        except Exception as e:
            print(f"An error occurred while writing the file: {e}")

