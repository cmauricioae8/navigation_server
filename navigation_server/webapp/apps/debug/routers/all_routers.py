#!/usr/bin/env python

from typing import Union
from fastapi import APIRouter, status
import subprocess, re
import logging

from navigation_server.webapp.apps.base.serializers import SimpleResponse, SimpleResponseList, DataResponse

logger = logging.getLogger()

router = APIRouter()


@router.get(
    "/pm2/list/",
    response_model=Union[SimpleResponse,SimpleResponseList],
    status_code=status.HTTP_200_OK,
    summary="Get List of PM2 Processes",
)
def get_pm2_list() -> list[str]:
    """
    Get list of PM2 processes with information.
    """
    #check_ouptut (not Popen) is used to remove all the b (byte format)
    output_cmd_byte = subprocess.check_output([
        "pm2 l --sort id | grep default | awk '{print $4} {print $18} {print $20} {print $22}'"], shell=True, encoding='utf-8')
    
    N_proc = 4 # Since {print $X} was used N_proc times

    output_cmd_arr = output_cmd_byte.split()
    ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
    print(output_cmd_arr)
    pr_list = [ansi_escape.sub('', item) for item in output_cmd_arr]
    n_proc = len(pr_list)//N_proc
    
    if n_proc > 0:
        pm2_processes = []

        for i in range(n_proc):
            pr = {}
            pr['name'] = pr_list[i*N_proc]
            pr['status'] = str(pr_list[i*N_proc+1])
            pr['cpu'] = str(pr_list[i*N_proc+2])
            pr['mem'] = str(pr_list[i*N_proc+3])

            ## Remove backend and frontend processes from list
            if pr['name'] == 'navigation_server' or pr['name'] == 'teleop-frontend':
                continue
            pm2_processes.append(pr)
    else:
        return SimpleResponse(status="FAIL", message="No PM2 processes", error="No PM2 processes")

    # print(pm2_processes)
    return SimpleResponseList(status="OK", message=pm2_processes)


@router.get(
    "/pm2/save/",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Save PM2 Processes",
)
def save_pm2_processes():
    """
    Save Current PM2 Processes.
    """
    save_cmd = ["pm2","save"]
    sub_process = subprocess.Popen(save_cmd, stdout=subprocess.PIPE)
    # print(sub_process)
    return SimpleResponse(status="OK", message="Command executed")


@router.post(
    "/pm2/start/{name}",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Start a PM2 Process",
)
def save_pm2_processes(name: str):
    """
    Start a PM2 Process by name.
    """
    pm2_cmd = ["pm2","start",name]
    sub_process = subprocess.Popen(pm2_cmd, stdout=subprocess.PIPE)
    return SimpleResponse(status="OK", message="Command executed")


@router.post(
    "/pm2/stop/{name}",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Stop a PM2 Process",
)
def save_pm2_processes(name: str):
    """
    Stop a PM2 Process by name.
    """
    pm2_cmd = ["pm2","stop",name]
    if name != "naviagtion_Server":
        sub_process = subprocess.Popen(pm2_cmd, stdout=subprocess.PIPE)
        return SimpleResponse(status="OK", message="Command executed")
    else:
        return SimpleResponse(status="FAIL", message="Backend can not be stopped")


@router.get(
    "/ros/list/{type}",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Get ROS Topics/Nodes List",
)
def ros_list(type: str):
    """
    Get ROS {topic, node} List.
    """
    cmd_rt = ["ros2",type,"list"]
    ret_cmd_rt = subprocess.run(cmd_rt, stdout=subprocess.PIPE, encoding='utf-8')
    # print(ret_cmd_rt.stdout)
    return SimpleResponse(status="OK", message=ret_cmd_rt.stdout)


@router.get(
    "/ros/status/",
    response_model=SimpleResponse,
    status_code=status.HTTP_200_OK,
    summary="Get ROS Core Status",
)
def ros_node_list():
    """
    Get ROS Status by running core_status node.
    """
    logger.warning(f"Running core_status node, black_box node MUST be running")
    cmd_rt = ["ros2","run","mau_core","core_status","-nocolor"]
    ret_cmd_rt = subprocess.run(cmd_rt, stdout=subprocess.PIPE)
    # print(ret_cmd_rt.stdout)

    if ret_cmd_rt.stdout == b'':
        return SimpleResponse(status="FAIL", message="Service NOT available")
    else:
        return SimpleResponse(status="OK", message=ret_cmd_rt.stdout)


@router.get(
    "/robot/network/",
    response_model=DataResponse,
    status_code=status.HTTP_200_OK,
    summary="Get Robot Network Information",
)
def get_network_info():
    """
    Get robot network information.
    """

    network_info = {"name": ""}
    try:
        output0 = subprocess.check_output(['iwgetid']).decode()
        output1 = output0.split()[1]
        output = output1.replace("ESSID:","")
        # print("Connected Wifi SSID: " + output)
        output_status = "OK"
        output_msg = "Wi-Fi connection"
        network_info["name"] = output
    except:
        output_status = "OK"
        output_msg = "Ethernet connection"
        network_info["name"] = "enp2s0"
    
    return DataResponse(status=output_status, message=output_msg, data=network_info)