#!/usr/bin/env python

import time
import threading
import math
import logging
import subprocess

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


class NavigationManager:
    def __init__(self):
        self.client: NavStackClient = base_node.navstack_client
        self.client.feedback_callback = self.navigation_feedback_callback

        # Clear costmap service from terminal
        self.clear_map_cmd = ' '.join(['ros2','service','call',\
            '/local_costmap/clear_entirely_local_costmap','nav2_msgs/srv/ClearEntireCostmap',\
            'request:\\','{}'])
        
        self.mode = "stop"
        self.mode_ready = True

        self.on_navigation = False
        self.paused_navigation = False
        self.admin_pause = False #Required for admin control EP
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

        self.nav_distance_error = None
        self.nav_orientation_error = None


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
            
            # Conversion from id to waypoint type
            for stop_waypoint in self.stop_waypoints:
                stop_waypoint.waypoint = waypoint_crud.get(stop_waypoint.waypoint)
                # logger.warning(f"   new {stop_waypoint.waypoint=}")
            self.delivery_waypoint = None
        else:
            return False, "Waypoint or path is required"

        if self.delivery_waypoint: ## ---------------------------------
            laps = 1
            self.path = Path()
            self.path.name = "_Delivery"
            self.stop_waypoints = []

            delivery_stop_waypoint = StopWaypoint(self.delivery_waypoint)
            delivery_stop_waypoint.attempts = 0
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


        base_node.nav_distance_tol = base_node.get_parameter('nav_distance_tol').get_parameter_value().double_value
        base_node.nav_orientation_tol = base_node.get_parameter('nav_orientation_tol').get_parameter_value().double_value
        
        base_node.logger.info(
            f"""*Navigation*Navigation started for path: {self.path.name}, with 
            "distance_tol: {base_node.nav_distance_tol}, orient_tol: {base_node.nav_orientation_tol}"""
        )

        myThread = threading.Thread(target=self.loop_navigation)
        myThread.daemon = True
        myThread.start()

        self.supervisor_thread = threading.Thread(target=self.supervisor)
        self.supervisor_thread.daemon = True
        self.supervisor_thread.start()
        
        return True, "Navigation started"
    
    
    def loop_navigation(self):
        logger.warning("start navigation loop thread for path: {}".format(self.path.name))

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
                        # Clear local costmap before starting
                        p = subprocess.Popen(self.clear_map_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
                        stdoutdata, stderrdata = p.communicate()  #this is blocking
                        # base_node.logger.info(f"{stdoutdata.decode()}---------")
                        
                        # send the goal and wait
                        self.state = self.send_waypoint_and_wait(stop_waypoint.waypoint)

                        self.message = self.state.description
                        logger.warning(
                            f""" End wait goal, state: {self.state.name}, message: {self.message}"""
                        )

                    # check if navigation is paused
                    if self.paused_navigation:
                        logger.info("Navigation paused")
                        self.state = NavigationState.PAUSED
                        self.message = "Navigation paused"
                        self.send_status_event()
                        base_node.logger.info("*Navigation*Navigation paused")

                        base_node.action_in_paused = base_node.get_parameter('action_in_paused').get_parameter_value().bool_value
                        if base_node.action_in_paused:
                            base_node.rotate2person_cmd_publisher.publish(True) # Activate node when paused
                            base_node.logger.info("*Navigation*Action in paused initialized")
                        
                        # wait for resume navigation (blocking)
                        while self.paused_navigation and self.on_navigation:
                            time.sleep(2.0)
                        
                        if base_node.action_in_paused:
                            base_node.rotate2person_cmd_publisher.publish(False) # Deactivate node
                            base_node.logger.info("*Navigation*Action in paused finalized")
                        
                        logger.info("Navigation resumed (or cancelled)")
                        self.state = NavigationState.ACTIVE
                        self.message = "On route"
                        self.send_status_event()
                        base_node.logger.info("*Navigation*Navigation resumed (or cancelled)")
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
                    elif self.state != NavigationState.SUCCEEDED: ## -----------
                        base_node.logger.warning(f"*Navigation*{self.state}")
                        pass
                    # check if navigation reached waypoint
                    elif (
                        self.state == NavigationState.SUCCEEDED or self.near_to_waypoint
                    ):
                        logger.info("Waypoint reached")
                        self.on_waypoint = True
                        # publish event on websocket to notify
                        self.send_status_event()
                        # Wait for the next stop waypoint
                        time.sleep(
                            0.01
                            if stop_waypoint.stop_time is None
                            else stop_waypoint.stop_time
                        )
                        self.on_waypoint = False
                        # break loop for receive pause/resume navigation
                        break

                    # check if attempts limit reached
                    if (
                        stop_waypoint.attempts > 0
                        and self.next_stop_waypoint_attempts  # attempts limit
                        >= stop_waypoint.attempts  # attempts limit reached
                    ):
                        base_node.logger.warning(
                            f"*Navigation* Cant reach waypoint {stop_waypoint.waypoint.name}, go to next")
                        break

                    # if cant reach waypoint, try again
                    base_node.logger.info(
                        "*Navigation*" f"Cant reach waypoint {stop_waypoint.waypoint.name}, try again"
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
            self.nav_distance_error = None #----------
            self.nav_orientation_error = None
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
        base_node.logger.info(f"*Navigation*Navigation finished for path: {self.path.name}")
        # publish event on websocket to notify
        self.send_status_event()
        self.on_navigation = False # ----------------

    
    def supervisor(self):
        time.sleep(2.0)
        logger.warning("Start navigator supervisor")

        while self.on_navigation:
            time.sleep(1.0)
            # continue if navigation is not active
            if self.state != NavigationState.ACTIVE:
                continue
            
            # update ros params
            base_node.nav_distance_tol = base_node.get_parameter('nav_distance_tol').get_parameter_value().double_value
            base_node.nav_orientation_tol = base_node.get_parameter('nav_orientation_tol').get_parameter_value().double_value

            wp_name = self.next_stop_waypoint.waypoint.name

            if self.nav_distance_error is not None:
                base_node.logger.warning("<<<<<<<<< Nav feedback can be used <<<<<<<<<<")
                base_node.logger.info(f"{self.nav_distance_error=}, {self.nav_orientation_error=}")

            ## DISTANCE AND ORIENTATION ARE VALIDATED
            if ( self.nav_distance_error < base_node.nav_distance_tol 
                and abs(self.nav_orientation_error) < base_node.nav_orientation_tol
            ):
                base_node.logger.info(
                    f"*Navigation*Near to waypoint {wp_name}, Supervisor cancels goal"
                )
                self.client.cancel_goal()
                self.near_to_waypoint = True
            
            ## TODO: Frontal_free verification for lane follower should be here
            ## TODO: implement time supervisor by checking odometry/cmd_vel for motion, not on_moving

        logger.warning("End navigator supervisor")
    

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

            self.nav_distance_error = euclidean_distance
            self.nav_orientation_error = current_orientation - self.client.goal_orientation

            doz = abs(self.nav_orientation_error)

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
    

    def on_fail_reach_wp(self):
        pass

    def cancel_navigation(self):
        if self.on_navigation:
            self.client.cancel_goal()
            self.on_navigation = False
            self.paused_navigation = False
            return True
        else:
            return False
    
    def pause_navigation(self):
        if self.on_navigation:
            self.client.cancel_goal()
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
    

    def send_waypoint_and_wait(self, waypoint: Waypoint):
        self.timeout_reached = False
        self.cant_reach_wp = False
        self.client.count_lost = 0
        self.next_stop_waypoint_attempts += 1
        self.next_stop_waypoint_start_time = time.time()

        self.client.send_goal(waypoint.position_x,waypoint.position_y,waypoint.orientation)

        self.state = NavigationState.ACTIVE
        self.message = "On route"

        self.send_status_event() # publish event on websocket to notify
        return self.client.wait_until_task_complete() # wait for goal response
    
    
    def get_navigation_status(self) -> NavigationStatusSerializer:
        next_waypoint_format = None
        if self.next_stop_waypoint is not None:
            logger.warning(f"*Navigation*Going to waypoint: {self.next_stop_waypoint.waypoint.name}")
            next_waypoint_format = WaypointSimplestSerializer(self.next_stop_waypoint.waypoint)
            
        return NavigationStatusSerializer(
            on_navigation=self.on_navigation,
            paused_navigation=self.paused_navigation,
            path=PathSimplestSerializer(self.path) if self.path else None,
            mode=self.mode,
            laps=self.laps,
            lap=self.lap,
            on_waypoint=self.on_waypoint,
            next_waypoint=next_waypoint_format, #
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


navigation_manager = NavigationManager()