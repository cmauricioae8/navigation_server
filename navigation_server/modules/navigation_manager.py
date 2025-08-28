#!/usr/bin/env python

import time
import threading
import math
import logging

# Models
from navigation_server.webapp.apps.paths.cruds.path_cruds import path_crud
from navigation_server.webapp.apps.paths.models import Path, StopWaypoint
from navigation_server.webapp.apps.waypoints.cruds.waypoint_cruds import (
    waypoint_crud,
)
from navigation_server.webapp.apps.waypoints.models import Waypoint

# Serializers
from navigation_server.modules.navigation_manager_models import NavigationState
from navigation_server.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationStatusSerializer,
)
from navigation_server.webapp.apps.paths.serializers.path_serializers import (
    PathSimplestSerializer,
)
from navigation_server.webapp.apps.waypoints.serializers.waypoint_serializers import (
    WaypointSimplestSerializer,
)
from navigation_server.modules.navigation_manager_models import PathMode

# Local apps
from navigation_server.clients.navstack_client import NavStackClient
from navigation_server.utils import euler_from_quaternion
from navigation_server.webapp.socket_io import emitEvent

from navigation_server.base_node import base_node # For ROS logger
logger = logging.getLogger("nav_manager") # For back-end logger

# Params that must be included as a ros params in base node
DISTANCE_ERROR = 0.4
ENABLE_GOAL_SUPERVISOR = True


class NavigationManager:
    def __init__(self):
        self.client: NavStackClient = base_node.navstack_client
        self.client.feedback_callback = self.navigation_feedback_callback

        self.mode = "stop"
        self.mode_ready = True

        self.on_navigation = False
        self.paused_navigation = False
        self.path = None
        self.laps = 0
        self.lap = 0
        self.state = NavigationState.READY
        self.message = ""

        self.next_stop_waypoint = None
        self.next_stop_waypoint_attempts = 0
        self.next_stop_waypoint_start_time = 0
        self.on_waypoint = False

        self.supervisor_thread = None
        self.near_to_waypoint = False
        self.timeout_reached = False
        self.cant_reach_wp = False


    def start_navigation(
        self, path_id: int, waypoint_id: int, mode: PathMode, laps: int
    ):
        if self.on_navigation:
            return False, "Navigation is already on"

        if waypoint_id is not None and waypoint_id > 0:
            self.delivery_waypoint: Waypoint = waypoint_crud.get(waypoint_id)
        elif path_id is not None and path_id > 0:
            self.path: Path = path_crud.get(path_id)
            self.stop_waypoints: list[StopWaypoint] = self.path.stop_waypoints()
            self.delivery_waypoint = None
        else:
            return False, "Waypoint or path is required"

        if self.delivery_waypoint: ## ---------------------------------
            laps = 1
            self.path = Path()
            self.path.name = "Delivery"
            self.stop_waypoints = []
            delivery_stop_waypoint = StopWaypoint(self.delivery_waypoint.id)
            delivery_stop_waypoint.attempts = 1
            delivery_stop_waypoint.time_attempt = 0
            delivery_stop_waypoint.stop_time = 0.0
            self.stop_waypoints.append(delivery_stop_waypoint)

        elif self.path and len(self.path.stop_waypoints()) < 2:
            message = "The path {} does not have enough stop_waypoints".format(
                self.path
            )
            logger.warning(message)
            return False, message

        base_node.stop_waypoints = self.stop_waypoints

        self.mode = mode
        self.laps = laps
        self.next_stop_waypoint_attempts = 0

        myThread = threading.Thread(target=self.loop_navigation)
        myThread.daemon = True
        myThread.start()

        self.supervisor_thread = threading.Thread(target=self.supervisor)
        self.supervisor_thread.daemon = True
        self.supervisor_thread.start()

        base_node.logger.info(
            f"*Navigation*Navigation started for path: {self.path.name}"
        )

        return True, "Navigation started"
    

    def loop_navigation(self):
        logger.info("start navigation loop thread for path: {}".format(self.path))

        # set initial state
        self.on_navigation = True
        self.lap = 0
        self.next_stop_waypoint = None
        self.on_waypoint = False

        self.client.wait_until_ready()

        # start navigation loop
        while self.on_navigation:
            # for each waypoint in path
            for stop_waypoint in self.stop_waypoints:
                waypoint: Waypoint = waypoint_crud.get(stop_waypoint.waypoint)
                # new stop waypoint
                self.next_stop_waypoint = stop_waypoint
                self.next_stop_waypoint_attempts = 0
                self.near_to_waypoint = False
                self.timeout_reached = False
                self.cant_reach_wp = False
                self.client.count_lost = 0
                # loop for to receive pause/resume navigation or new attempt
                while self.on_navigation:
                    # send goal if navigation is not paused
                    if not self.paused_navigation:
                        logger.info("----------------------------------------")
                        logger.info("for stop_waypoint: {}".format(str(stop_waypoint)))
                        # send the goal and wait
                        self.state = self.send_waypoint_and_wait(waypoint)

                        self.message = self.state.description
                        logger.info(
                            f""" End wait goal, state: {self.state.name}, message: {self.message}"""
                        )

                    # check if navigation is paused
                    if self.paused_navigation:
                        logger.info("Navigation paused")
                        self.state = NavigationState.PAUSED
                        self.message = "Navigation paused"
                        self.send_status_event()
                        base_node.logger.info("*Navigation*Navigation paused")
                        # wait for resume navigation
                        while self.paused_navigation and self.on_navigation:
                            time.sleep(2.0)
                        logger.info("Navigation resumed (or cancelled)")
                        self.state = NavigationState.ACTIVE
                        self.message = "On route"
                        self.send_status_event()
                        base_node.logger.info(
                            "*Navigation*Navigation resumed (or cancelled)"
                        )
                        continue
                    # check if navigation was cancelled
                    elif not self.on_navigation:
                        logger.info("Navigation was cancelled")
                        base_node.logger.info("*Navigation*Navigation was cancelled")
                        break
                    elif self.timeout_reached:
                        logger.warning("Timeout reached")
                        pass
                    elif self.cant_reach_wp:
                        logger.warning("Robot cant reach waypoint")
                        pass
                    elif self.state != NavigationState.SUCCEEDED:
                        logger.warning(f"Navigation failed: {self.state}")
                        base_node.logger.warning(
                            f"*Navigation*Navigation failed: {self.state}"
                        )
                        pass
                    # check if navigation reached waypoint
                    elif (
                        self.state == NavigationState.SUCCEEDED or self.near_to_waypoint
                    ):
                        logger.info("Navigation reached waypoint")
                        self.on_waypoint = True
                        # publish event on websocket to notify
                        self.send_status_event()
                        # Wait for the next stop waypoint
                        time.sleep(
                            0.01
                            if stop_waypoint.stop_time is None
                            else stop_waypoint.stop_time
                        )
                        # execute action
                        # for action in stop_waypoint.actions:
                        #     self.execute_action(action)
                        self.on_waypoint = False
                        # break loop for receive pause/resume navigation
                        break

                    # check if attempts limit reached
                    if (
                        stop_waypoint.attempts != 0
                        and self.next_stop_waypoint_attempts  # attempts limit
                        >= stop_waypoint.attempts  # attempts limit reached
                    ):
                        logger.warning(
                            f"Cant reach waypoint {waypoint.name}, " "go to next"
                        )
                        base_node.logger.warning(
                            f"*Navigation* Cant reach waypoint {waypoint.name}, go to next"
                        )
                        break

                    # if cant reach waypoint, try again
                    logger.info("Cant reach waypoint, try again")
                    base_node.logger.info(
                        "*Navigation*" f"Cant reach waypoint {waypoint.name}, try again"
                    )
                    time.sleep(2.0)

                # if navigation is canceled or failed, stop waypoint loop
                if not self.on_navigation:
                    break

            # update variables for next lap
            self.lap += 1
            logger.info("Lap: {}".format(self.lap))
            if self.laps == -1 and self.mode != PathMode.ONCE:
                pass
            elif self.lap >= self.laps or self.mode == PathMode.ONCE:
                logger.info("Reach the end of the path")
                self.on_navigation = False
                break
            if self.mode == PathMode.REVERSE_LOOP:
                self.stop_waypoints = self.stop_waypoints[::-1]
                logger.info("Reverse stop_waypoints")
        # end navigation loop

        # stop supervisor thread
        try:
            logger.info("stop navigation supervisor")
            self.supervisor_thread.stop()
        except Exception:
            logger.warning("navigation supervisor was not running")

        self.state = NavigationState.SUCCEEDED
        self.message = "Navigation finished"
        self.next_stop_waypoint = None

        # Stop robot after a path finished
        logger.info("Stop robot after a path finished")
        base_node.cmd_vel_publisher.publish(0.0, 0.0)

        logger.info("end navigation loop thread for path: {}".format(self.path))
        # publish event on websocket to notify
        self.send_status_event()
        base_node.logger.info(
            f"*Navigation*Navigation finished for path: {self.path.name}"
        )

    
    def supervisor(self):
        logger.info(
            f"Start navigator supervisor, with distance error: {DISTANCE_ERROR}")

        while self.on_navigation:
            time.sleep(1.0)
            # continue if navigation is not active
            if self.state != NavigationState.ACTIVE:
                continue

            # goal supervisor
            if ENABLE_GOAL_SUPERVISOR:
                # get current position
                current_x = base_node.amcl_pose_subscriber.pose_data.position_x
                current_y = base_node.amcl_pose_subscriber.pose_data.position_y

                # get waypoint position
                logger.warning(f"---------------{self.next_stop_waypoint=}")
                waypoint_x = self.next_stop_waypoint.waypoint.position_x
                waypoint_y = self.next_stop_waypoint.waypoint.position_y
                wp_name = self.next_stop_waypoint.waypoint.name

                # get distance between current and waypoint
                distance = math.sqrt(
                    (current_x - waypoint_x) ** 2 + (current_y - waypoint_y) ** 2
                )
                ## ONLY DISTANCE IS VALIDATED, NOT ORIENTATION --------------
                if distance < DISTANCE_ERROR:
                    logger.warning(f"Near to waypoint {wp_name}, cancel goal")
                    base_node.logger.warning(
                        f"*Navigation*Near to waypoint {wp_name}, cancel goal"
                    )
                    self.client.cancel_goal()
                    self.near_to_waypoint = True

            # time supervisor
            ## Cancel a goal by time is not recommended -------------
            if ( self.next_stop_waypoint.time_attempt > 0
                and (time.time() - self.next_stop_waypoint_start_time)
                > self.next_stop_waypoint.time_attempt
            ):
                logger.warning(
                    f"Time limit reached for waypoint {wp_name}, cancel goal"
                )
                base_node.logger.warning(
                    "*Navigation*"
                    f"Time limit reached for waypoint {wp_name}, "
                    "cancel goal"
                )
                self.client.cancel_goal()
                self.timeout_reached = True

        logger.info("End navigator supervisor")

    
    def cancel_navigation(self):
        if self.on_navigation:
            self.client.cancel_all_goals()
            self.on_navigation = False
            self.paused_navigation = False
            return True
        else:
            return False

    def pause_navigation(self):
        if self.on_navigation:
            self.client.cancel_goal()
            # self.client.cancel_all_goals()
            self.paused_navigation = True
            return True
        else:
            return False

    def resume_navigation(self):
        if self.on_navigation:
            self.paused_navigation = False
            self.next_stop_waypoint_attempts = 0
            return True
        else:
            return False

    def cancel_goal(self):
        if self.on_navigation:
            logger.info("cancel goal")
            self.client.cancel_goal()
            return True
        else:
            return False

    def on_fail_reach_wp(self):
        pass


    def navigation_feedback_callback(self, feedback_msg):
        # calculate every second
        if (time.time() - self.client.last_t0) > 1.0:
            # for current pose
            current_pose = feedback_msg.feedback.current_pose.pose
            current_position_x = current_pose.position.x
            current_position_y = current_pose.position.y
            current_orientation = euler_from_quaternion(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            )[2]

            euclidean_distance = math.sqrt(
                (current_position_x - self.client.goal_position_x) ** 2
                + (current_position_y - self.client.goal_position_y) ** 2
            )
            doz = abs(current_orientation - self.client.goal_orientation)

            eta = (
                feedback_msg.feedback.estimated_time_remaining.sec
                + feedback_msg.feedback.estimated_time_remaining.nanosec / 1e9
            )
            dr = feedback_msg.feedback.distance_remaining
            tt = (
                feedback_msg.feedback.navigation_time.sec
                + feedback_msg.feedback.navigation_time.nanosec / 1e9
            )
            r = feedback_msg.feedback.number_of_recoveries
            delta_dr = (self.client.last_dr - dr) / (self.client.last_t0 - time.time())
            delta_doz = abs(self.client.last_doz - doz)
            self.client.last_dr = dr
            self.client.last_doz = doz
            self.client.last_t0 = time.time()

            if delta_dr > -0.01 and delta_doz < 0.1:
                self.client.count_lost += 1
            else:
                self.client.count_lost = 0

            # Uncomment to enable supervisor when robot cant reach waypoint
            """
            if (
               self.client.count_lost >= 20
               and self.next_stop_waypoint.attempts == 0
               and not self.paused_navigation
               and euclidean_distance < 1.5
            ):
               logger.warning("The robot cant move, pause navigation")
               self.pause_navigation()
               self.on_fail_reach_wp(self.paused_navigation)

            if (
               self.client.count_lost >= 7
               and self.next_stop_waypoint.attempts > 0
               and not self.cant_reach_wp
               # and euclidean_distance < 1.5
            ):
               logger.warning(
                   "The robot not moving, cancel current goal and go to next"
               )
               self.client.cancel_goal()
               self.cant_reach_wp = True
               self.on_fail_reach_wp(False)
            """

            logger.info(
                f"NavCFb ETA:{eta:2.1f} s, "
                f"Dr:{dr:2.2f} m, "
                f"Deu:{euclidean_distance:2.2f} m, "
                f"doz: {doz:2.1f} rad, "
                f"Tt:{tt:2.1f} s, "
                f"R:{r},\t "
                f"V:{delta_dr:2.2f} m/s, "
                f"d_doz:{delta_doz:2.1f} rad/s, "
                f"cl:{self.client.count_lost}"
            )


    def get_navigation_status(self) -> NavigationStatusSerializer:
        waypoint = None
        if self.next_stop_waypoint:
            waypoint = waypoint_crud.get(self.next_stop_waypoint.waypoint)
            # logger.warning(f"-------------------- {waypoint=}")
        return NavigationStatusSerializer(
            on_navigation=self.on_navigation,
            paused_navigation=self.paused_navigation,
            path=PathSimplestSerializer(self.path) if self.path else None,
            mode=self.mode,
            laps=self.laps,
            lap=self.lap,
            on_waypoint=self.on_waypoint,
            next_waypoint=WaypointSimplestSerializer(waypoint) if waypoint else None,
            attempt=self.next_stop_waypoint_attempts,
            start_time=self.next_stop_waypoint_start_time,
            state=self.state,
            message=self.message,
        )
    

    def send_status_event(self):
        emitEvent(
            "on_status_change",
            {"data": {"navigation": self.get_navigation_status().to_dict()}},
        )

    
    def send_waypoint_and_wait(self, waypoint: Waypoint):
        self.timeout_reached = False
        self.cant_reach_wp = False
        self.client.count_lost = 0
        self.next_stop_waypoint_attempts += 1
        self.next_stop_waypoint_start_time = time.time()

        self.client.send_goal(waypoint.position_x,waypoint.position_y,waypoint.orientation)

        self.state = NavigationState.ACTIVE
        self.message = "On route"

        # publish event on websocket to notify
        self.send_status_event()

        # wait for goal response
        return self.client.wait_until_task_complete()


navigation_manager = NavigationManager()