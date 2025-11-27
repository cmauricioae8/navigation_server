#!/usr/bin/env python

from typing import Union
from fastapi import APIRouter, status
import subprocess, re, builtins

from navigation_server.webapp.apps.base.serializers import SimpleResponse, DataResponse
from navigation_server.webapp.settings import SERVER_NODE_NAME
from navigation_server.webapp.apps.params.forms.params_forms import (
    SetRobotParamsForm,
    SetSensorProtectForm,
    SetNavParamsForm,
)

router = APIRouter()


@router.get(
    "/robot_params/",
    response_model=Union[SimpleResponse,DataResponse],
    status_code=status.HTTP_200_OK,
    summary="Get ROS Parameters of robot",
)
async def get_robot_params():
    """
    Get ROS Parameters of robot that user can modified.
    """
    cmd = ["ros2","param","dump","/octy_safe_motion"]
    ret_cmd_rt = subprocess.run(cmd, stdout=subprocess.PIPE)
    output = str(ret_cmd_rt.stdout)

    clean_output = output.strip().strip("b'").replace('\\n', '\n').replace('\\t', '\t')
    # print(clean_output)

    octy_save_params = [
        {"name": "max_vel_x", "type": "float"}, {"name": "min_vel_x", "type": "float"},
        {"name": "max_vel_theta", "type": "float"}, {"name": "manual_vel_gain", "type": "float"},
    ]

    params_dict = {}

    # Iterate through the desired parameters and find their values using regex
    for param in octy_save_params:
        # Regex pattern:
        # ^[ \t]* -> Match start of line (^) followed by optional spaces/tabs
        # (param) -> Capture the parameter name (key)
        # :[ \t]* -> Match the colon and optional spaces/tabs
        # (.*)    -> Capture the rest of the line (the value)
        pattern = rf"^[ \t]*({re.escape(param['name'])}):[ \t]*(.*)$"

        # Search the clean output string line by line
        match = re.search(pattern, clean_output, re.MULTILINE)

        if match:
            # Group 1 is the parameter name, Group 2 is the value
            key = match.group(1)
            value = match.group(2).strip() # Strip any trailing whitespace/newlines (string type)

            ## Dynamic cast
            type_function = getattr(builtins, param['type'])
            var_type = type_function(value)
            
            params_dict[key] = var_type
    

    # print(params_dict)
    if not params_dict:
        return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    else:
        return DataResponse(status="OK", message="ROS params", data=params_dict)
    

@router.get(
    "/sensor_protect/",
    response_model=Union[SimpleResponse,DataResponse],
    status_code=status.HTTP_200_OK,
    summary="Get ROS Parameters of sensors protection",
)
async def get_sensor_protect():
    """
    Get ROS Parameters of sensors protection that user can modified.
    """
    cmd = ["ros2","param","dump","/octy_safe_motion"]
    ret_cmd_rt = subprocess.run(cmd, stdout=subprocess.PIPE)
    output = str(ret_cmd_rt.stdout)

    clean_output = output.strip().strip("b'").replace('\\n', '\n').replace('\\t', '\t')
    # print(clean_output)
    octy_save_params = [
        {"name": "use_lidar", "type": "bool"}, {"name": "use_voltage", "type": "bool"},
        {"name": "use_sonars", "type": "bool"}, {"name": "use_imu", "type": "bool"},
    ]

    params_dict = {}

    for param in octy_save_params:
        pattern = rf"^[ \t]*({re.escape(param['name'])}):[ \t]*(.*)$"
        match = re.search(pattern, clean_output, re.MULTILINE)

        if match:
            key = match.group(1)
            value = match.group(2).strip() # Strip any trailing whitespace/newlines
            type_function = getattr(builtins, param['type'])
            var_type = type_function(value)
            params_dict[key] = var_type

    # print(params_dict)
    if not params_dict:
        return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    else:
        return DataResponse(status="OK", message="ROS params", data=params_dict)


@router.get(
    "/nav_manager/",
    response_model=Union[SimpleResponse,DataResponse],
    status_code=status.HTTP_200_OK,
    summary="Get ROS Parameters of navigation manager",
)
async def get_nav_manager():
    """
    Get ROS Parameters of navigation manager that user can modified.
    """
    cmd = ["ros2","param","dump","/"+SERVER_NODE_NAME]
    ret_cmd_rt = subprocess.run(cmd, stdout=subprocess.PIPE)
    output = str(ret_cmd_rt.stdout)

    clean_output = output.strip().strip("b'").replace('\\n', '\n').replace('\\t', '\t')
    # print(clean_output)
    nav_params = [
        {"name": "action_in_paused", "type": "bool"},
        {"name": "nav_distance_tol", "type": "float"}, {"name": "nav_orientation_tol", "type": "float"},
    ]

    params_dict = {}

    for param in nav_params:
        pattern = rf"^[ \t]*({re.escape(param['name'])}):[ \t]*(.*)$"
        match = re.search(pattern, clean_output, re.MULTILINE)
        if match:
            key = match.group(1)
            value = match.group(2).strip() # Strip any trailing whitespace/newlines
            type_function = getattr(builtins, param['type'])
            var_type = type_function(value)
            params_dict[key] = var_type
    
    # print(params_dict)
    if not params_dict:
        return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    else:
        return DataResponse(status="OK", message="ROS params", data=params_dict)


def run_command(param_name: str, param_value: str, node_name = "/octy_safe_motion") -> str:
    """
    Recurrent function to run a terminal command (set a ROS param) as subprocess.
    """
    cmd = ["ros2","param","set",node_name,param_name,param_value]
    ret_cmd_rt = subprocess.run(cmd, stdout=subprocess.PIPE)
    output = str(ret_cmd_rt.stdout)
    clean_output = output.strip().strip("b'").replace('\\n', '\n').replace('\\t', '\t')
    # print(clean_output)
    if clean_output == "Node not found": return False
    else: return True

@router.post(
    "/robot_params/",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Set ROS Parameters of robot",
)
async def set_robot_params(robot_param: SetRobotParamsForm):
    """
    Set ROS Parameters of robot that user can modified.
    """
    # print(robot_param)

    if robot_param.max_vel_x is not None:
        response = run_command('max_vel_x',str(robot_param.max_vel_x))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")

    if robot_param.min_vel_x is not None:
        response = run_command('min_vel_x',str(robot_param.min_vel_x))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    
    if robot_param.max_vel_theta is not None:
        response = run_command('max_vel_theta',str(robot_param.max_vel_theta))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    
    if robot_param.manual_vel_gain is not None:
        response = run_command('manual_vel_gain',str(robot_param.manual_vel_gain))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")

    robot_param.save_to_ros_yaml() #Over-write yaml file
    return SimpleResponse(status="OK", message="ROS param set command executed")


@router.post(
    "/sensor_protect/",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Set ROS Parameters of sensor protection",
)
async def set_sensor_protect(sensor_protect: SetSensorProtectForm):
    """
    Set ROS Parameters of sensor protection that user can modified.
    """
    # print(sensor_protect)

    if sensor_protect.use_lidar is not None:
        response = run_command('use_lidar',str(sensor_protect.use_lidar))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")

    if sensor_protect.use_voltage is not None:
        response = run_command('use_voltage',str(sensor_protect.use_voltage))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    
    if sensor_protect.use_sonars is not None:
        response = run_command('use_sonars',str(sensor_protect.use_sonars))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")
    
    if sensor_protect.use_imu is not None:
        response = run_command('use_imu',str(sensor_protect.use_imu))
        if not response:
            return SimpleResponse(status="FAIL", message="octy_safe_motion node NOT found")

    sensor_protect.save_to_ros_yaml() #Over-write yaml file
    return SimpleResponse(status="OK", message="ROS param set command executed")


@router.post(
    "/nav_manager/",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Set ROS Parameters of navigation manager",
)
async def set_nav_manager(nav_man_params: SetNavParamsForm):
    """
    Set ROS Parameters of navigation manager that user can modified.
    """
    # print(nav_man_params)

    if nav_man_params.nav_distance_tol is not None:
        response = run_command('nav_distance_tol',str(nav_man_params.nav_distance_tol),"/"+SERVER_NODE_NAME)
        if not response:
            return SimpleResponse(status="FAIL", message="Could NOT modified ROS param of "+SERVER_NODE_NAME)
    
    if nav_man_params.nav_orientation_tol is not None:
        response = run_command('nav_orientation_tol',str(nav_man_params.nav_orientation_tol),"/"+SERVER_NODE_NAME)
        if not response:
            return SimpleResponse(status="FAIL", message="Could NOT modified ROS param of "+SERVER_NODE_NAME)

    if nav_man_params.action_in_paused is not None:
        response = run_command('action_in_paused',str(nav_man_params.action_in_paused),"/"+SERVER_NODE_NAME)
        if not response:
            return SimpleResponse(status="FAIL", message="Could NOT modified ROS param of "+SERVER_NODE_NAME)
    
    nav_man_params.save_to_ros_yaml() #Over-write yaml file
    return SimpleResponse(status="OK", message="ROS param set command executed")

