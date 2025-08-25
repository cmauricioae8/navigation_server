#!/usr/bin/env python
import copy
import subprocess
from subprocess import Popen
import time
import psutil
import threading
from os import path, environ

from navigation_server.webapp.apps.maps.models import Map
from navigation_server.webapp.apps.maps.cruds.map_cruds import map_crud_manager
from navigation_server.webapp.socket_io import emitEvent
from navigation_server.webapp.settings import OperationMode
from navigation_server.webapp.settings import settings, APP_DATA_DIR
from navigation_server.topics.pose_subscriber import RobotPoseData
from navigation_server.base_node import base_node
# from navigation_server.modules.navigation_manager import navigation_manager


class ModeManager(threading.Thread):
    SET_SAME_POSE = "same"
    SET_LAST_MODE_POSE = "last_mode"
    # NO_SET_POSE = ""

    def __init__(self):
        threading.Thread.__init__(self)

        self.desired_mode: OperationMode = OperationMode.STOP
        self.mode: OperationMode = OperationMode.STOP
        self.mode_ready = True
        self.map_id = 0
        self.map = None
        self.with_keepout_zones = False
        self.with_speed_limits = False

        self.processes = []

        self.last_mode_pose: RobotPoseData = None
        self.currently_changing_mode = False

        self.daemon = True

    def set_mode(
        self,
        mode: OperationMode,
        map_id=0,
        with_keepout_zones=False,
        with_speed_limits=False,
    ):
        # base_node.logger.info("inicio del set_mode------------------------------------------------")
        self.map_id = map_id if map_id is not None else 0
        self.map_name = None
        self.with_keepout_zones = with_keepout_zones
        self.with_speed_limits = with_speed_limits
        if self.map_id > 0:
            self.map: Map = map_crud_manager.get(self.map_id)
            
            if self.map is None:
                return False, "Map not found"
            self.map_name = self.map.name
            # for keepout zones
            # if not self.map.pgm_keepout_zones_path and self.with_keepout_zones:
            #     self.map.generate_files_with_keepout_zones()
            # for speed limits
            # if not self.map.pgm_speed_limits_path and self.with_speed_limits:
            #     self.map.generate_files_with_speed_limits()
        else:
            self.map = None
            self.map_name = "None"

        # if desired mode already is current mode, do nothing
        if self.mode == mode and self.mode_ready:
            # publish event with mode status
            self.send_status_event()
            return True, "Mode already is {}".format(mode)

        base_node.logger.info(
            (
                f"Change mode from '{self.mode}' to '{mode}', map_id: {self.map_id}, "
                f"map name: {self.map_name}, "
                f"with_keepout_zones: {self.with_keepout_zones}, "
                f"with_speed_limits: {self.with_speed_limits}"
            )
        )

        if not self.mode_ready and self.currently_changing_mode:
            return (
                False,
                "ModeManager is not ready to change mode, or is already changing mode",
            )

        self.desired_mode = mode
        self.mode_ready = False

        return True, "changing mode to {}".format(mode)

    def kill_process(self, popen_instance: Popen):
        try:
            process = psutil.Process(popen_instance.pid)
            for proc in process.children(recursive=True):
                proc.kill()
            process.kill()
        except Exception as e:
            base_node.logger.error(f"Error killing process: {e}")

    def supervise_process(self, popen_instance: Popen):
        base_node.logger.info(
            "Start supervise process ------------------------------------------------"
        )
        while popen_instance.returncode is None:
            # handle output by direct access to stdout and stderr
            for line in popen_instance.stdout:
                emitEvent(
                    "on_process_output",
                    {"data": {"output": line.decode("utf-8").strip()}},
                )
            # set returncode if the process has exited
            popen_instance.poll()
        base_node.logger.info(
            "End supervise process ------------------------------------------------"
        )

    def processes_add_command(self, env: dict, command: str):
        env = {**environ, **env}
        process = Popen(
            command.split(), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        )
        supervisor_thread = threading.Thread(
            target=self.supervise_process, args=(process,)
        )
        supervisor_thread.daemon = True
        supervisor_thread.start()
        self.processes.append(
            {"process": process, "supervisor_thread": supervisor_thread}
        )

    def processes_stop(self):
        if len(self.processes) > 0:
            base_node.logger.info("Stopping processes ...")
            for process_item in self.processes:
                self.kill_process(process_item["process"])
            for process_item in self.processes:
                process_item["process"].wait()
            base_node.logger.info("                   ... Processes stopped")
            self.processes = []
        else:
            base_node.logger.info("No processes to stop")

    def run(self):
        current_pose: RobotPoseData = None

        self.desired_mode = OperationMode.STATIC
        self.mode_ready = False
        base_node.logger.info("ModeManager thread to manage processes started")
        while True:
            # if no change in mode, do nothing
            if self.mode == self.desired_mode:
                time.sleep(1)
                continue

            self.currently_changing_mode = True

            # cancel navigation and stop robot
            # navigation_manager.cancel_navigation() #--------------------------------
            base_node.cmd_vel_publisher.publish(0.0, 0.0)

            # get last pose
            if base_node.pose_subscriber.pose_available:
                self.last_mode_pose = copy.deepcopy(base_node.pose_subscriber.pose_data)
                base_node.logger.info(
                    f"""Last mode pose:
                        position_x: {self.last_mode_pose.position_x:0.2f},
                        position_y: {self.last_mode_pose.position_y:0.2f},
                        orientation: {self.last_mode_pose.orientation:0.2f}"""
                )
            # unsubscribe to pose topic
            base_node.pose_subscriber.try_unsubscribe()

            # stop current mode by processes if exist
            self.processes_stop()

            # Stop robot after stop processes
            base_node.cmd_vel_publisher.publish(0.0, 0.0)

            # clear map
            base_node.map_subscriber.clear_map()
            base_node.costmap_subscriber.clear_map()

            success_mode_startup = False
            # if mode is self.STOP_MODE, mode start success
            if self.desired_mode == OperationMode.STOP:
                success_mode_startup = True


            # Start Attempts to change mode
            attempt_number = 0
            maximum_attempts = 3
            while( not success_mode_startup and attempt_number < maximum_attempts):
                attempt_number += 1
                base_node.logger.info(
                    f"---- Attempt to change mode #{attempt_number} "
                    f"from '{self.mode}' to '{self.desired_mode}' ----"
                )
                # stop processes
                self.processes_stop()

                # start desired mode
                # start processes
                process_set = settings.MODE_MANAGER.get_process(self.desired_mode)
                for command in process_set.COMMANDS:
                    if "{MAP_FILE}" in command and self.map_id > 0:
                        try:
                            map_fullpath = path.join(
                                APP_DATA_DIR, "media/", self.map.yaml_path
                            )
                            base_node.logger.info("Map file: {}".format(map_fullpath))
                            command = command.replace("{MAP_FILE}", map_fullpath)
                        except Exception:
                            base_node.logger.warning(
                                f"Map file with id:{self.map_id} not found"
                            )
                            command = command.replace("{MAP_FILE}", "")

                    base_node.logger.info("Starting process: {}".format(command))
                    self.processes_add_command(process_set.ENVIRON_VARS, command)
                ##############################

                base_node.map_subscriber.try_subscribe()
                base_node.pose_publisher.set_pose(0.0, 0.0, 0.0)

                # start keepout_zones_process
                if self.with_keepout_zones:
                    command = settings.KEEPOUT_ZONES_PROCESS.COMMANDS[0]
                    env = settings.KEEPOUT_ZONES_PROCESS.ENVIRON_VARS
                    if "{MAP_FILE}" in command and self.map_id > 0:
                        try:
                            map_fullpath = path.join(
                                APP_DATA_DIR, "media/maps/", self.map.name + "_keepout.yaml"
                            )
                            base_node.logger.info("Map keepout file: {}".format(map_fullpath))
                            command = command.replace("{MAP_FILE}", map_fullpath)
                        except Exception:
                            base_node.logger.warning(
                                f"Map keepout file with id:{self.map_id} not found"
                            )
                            command = command.replace("{MAP_FILE}", "")
                    if len(command) > 0:
                        base_node.logger.info("Starting process: {}".format(command))
                        self.processes_add_command(env, command)

                # start speed_limits_process
                if self.with_speed_limits:
                    command = settings.SPEED_LIMITS_PROCESS.COMMANDS[0]
                    env = settings.SPEED_LIMITS_PROCESS.ENVIRON_VARS
                    if "{MAP_FILE}" in command and self.map_id > 0:
                        try:
                            map_fullpath = path.join(
                                APP_DATA_DIR, "media/maps/", self.map.name + "_speed.yaml"
                            )
                            base_node.logger.info("Map speed file: {}".format(map_fullpath))
                            command = command.replace("{MAP_FILE}", map_fullpath)
                        except Exception:
                            base_node.logger.warning(
                                f"Map speed file with id:{self.map_id} not found"
                            )
                            command = command.replace("{MAP_FILE}", "")
                    if len(command) > 0:
                        base_node.logger.info("Starting process: {}".format(command))
                        self.processes_add_command(env, command)

                # wait to pose topic to confirm start mode
                if self.desired_mode in (
                    OperationMode.WAYPOINTS, OperationMode.NAVIGATION, OperationMode.MAPPING
                ):
                    base_node.logger.info("Wait to pose topic ...")
                    base_node.pose_subscriber.try_subscribe()
                    timeout_wait_pose = 10.0
                    t0_wait_pose = time.time()
                    while not base_node.pose_subscriber.pose_available and (
                        (time.time() - t0_wait_pose) < timeout_wait_pose
                    ):
                        time.sleep(1.0)
                        # publish an initial pose to received the correct pose from amcl_robot_pose
                        base_node.pose_publisher.set_pose(0.0, 0.0, 0.0)

                    if not base_node.pose_subscriber.pose_available:
                        base_node.logger.warning(
                            "*Modes*"
                            f"Pose not received after waiting {timeout_wait_pose}s, "
                            "restart change mode"
                        )
                        base_node.pose_subscriber.try_unsubscribe()
                        continue
                    else:
                        base_node.logger.info("               ...  pose received")
                        current_pose = copy.deepcopy(
                            base_node.pose_subscriber.pose_data
                        )

                # wait to map topic to confirm start mode
                if (
                    self.desired_mode in (
                        OperationMode.MAPPING, OperationMode.WAYPOINTS, OperationMode.NAVIGATION
                    ) and self.map_id > 0
                ):
                    base_node.logger.info("Wait to map topic ...")
                    timeout_wait_map = 15.0
                    t0_wait_map = time.time()
                    while (not base_node.map_subscriber.map_available) and (
                        (time.time() - t0_wait_map) < timeout_wait_map
                    ):
                        time.sleep(1.0)
                    if not base_node.map_subscriber.map_available:
                        base_node.logger.warning(
                            "*Modes*"
                            f"Map not received after waiting {timeout_wait_map}s, "
                            "restart change mode"
                        )
                        base_node.map_subscriber.try_unsubscribe()
                        continue
                    else:
                        base_node.logger.info(
                            "             ... "
                            f"map received in {(time.time()-t0_wait_map):2.2f}s"
                        )
                        base_node.map_subscriber.try_unsubscribe()

                # if reach this point, mode start success
                success_mode_startup = True
                # End attempts to change mode

            # if cant change mode after many attempts
            if not success_mode_startup:
                out_msg = "Cant change from {} to {} after {} attempts".format(
                    self.mode, self.desired_mode, attempt_number
                )
                base_node.logger.error(f"*Modes*{out_msg}")

                self.desired_mode = self.mode
                self.mode_ready = True
                # publish event on websocket to notify
                self.send_status_event()
                self.currently_changing_mode = False
                continue

            # if mode is stop, do nothing
            if self.desired_mode == OperationMode.STOP:
                base_node.logger.info("On 'self.STOP_MODE' mode")
                self.mode = self.desired_mode
                self.mode_ready = True
                # publish event on websocket to notify
                self.send_status_event()
                self.currently_changing_mode = False
                continue

            base_node.logger.info("wait 2 seconds")
            time.sleep(2.0)

            # restore last pose
            if self.desired_mode == OperationMode.MAPPING:
                base_node.logger.info("NO SET POSE ON MAPPING")
            elif (
                settings.POSE_TO_SET == self.SET_LAST_MODE_POSE
                and self.last_mode_pose is not None
            ):
                base_node.logger.info(
                    f"""SET LAST MODE POSE with initial covariance:
                        position_x: {self.last_mode_pose.position_x:0.2f},
                        position_y: {self.last_mode_pose.position_y:0.2f},
                        orientation: {self.last_mode_pose.orientation:0.2f}"""
                )
                base_node.pose_publisher.set_pose(
                    self.last_mode_pose.position_x,
                    self.last_mode_pose.position_y,
                    self.last_mode_pose.orientation,
                )
            elif (
                settings.POSE_TO_SET == self.SET_SAME_POSE and current_pose is not None
            ):
                base_node.logger.info(
                    f"""SET SAME POSE with initial covariance:
                            position_x: {current_pose.position_x:0.2f},
                            position_y: {current_pose.position_y:0.2f},
                            orientation: {current_pose.orientation:0.2f}"""
                )
                base_node.pose_publisher.set_pose(
                    current_pose.position_x,
                    current_pose.position_y,
                    current_pose.orientation,
                )
            # elif settings.POSE_TO_SET == self.NO_SET_POSE:
            #     base_node.logger.warning("NO SET POSE")
            else:
                base_node.logger.warning(
                    "No exist last pose, SET POSE TO ZERO with initial covariance"
                )
                base_node.pose_publisher.set_pose(0.0, 0.0, 0.0)

            self.mode = self.desired_mode
            self.mode_ready = True
            # publish event on websocket to notify
            base_node.logger.info(f"*Modes*Mode changed to {self.mode.value}")
            self.send_status_event()
            self.currently_changing_mode = False

    def send_status_event(self):
        emitEvent(
            "on_status_change",
            {
                "data": {
                    "operation_mode": {
                        "mode": self.mode.value,
                        "ready": self.mode_ready,
                    }
                }
            },
        )
        emitEvent(
            "on_mode_change",
            {
                "mode": self.mode.value,
            },
        )
        base_node.mode_status_publisher.publish(self.mode.value)


mode_manager = ModeManager()
