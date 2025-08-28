#!/usr/bin/env python3

"""
@description: Script to define the NavigationClient class to be used for the navigation_manager.
A ROS action client is created to be available to set goal to the Nav Stack.
An example of use is included to validate the ROS action client.
"""

import logging
import time
import math

# Third apps
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# import tf_transformations

# Serializers
from navigation_server.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationState,
)
from navigation_server.utils import euler_from_quaternion, quaternion_from_euler


logger = logging.getLogger("navstack_client")


class NavStackClient:
    def __init__(self, node: Node):
        self.node = node

        self.goal_handle = None
        self.result_future = None
        self.last_dr = 0.0
        self.last_t0 = 0.0
        self.last_doz = 0.0
        self.count_lost = 0
        self.goal_position_x = 0.0
        self.goal_position_y = 0.0
        self.goal_orientation = 0.0
        self.status: NavigationState = NavigationState.READY

        self.nav_to_pose_client = None

    def try_create_client(self):
        if self.nav_to_pose_client is not None:
            return
        self.nav_to_pose_client = ActionClient(
            self.node, NavigateToPose, "navigate_to_pose"
        )
        logger.info("NavigationClient has been created.")

    # public methods
    def wait_until_ready(self):
        logger.info("Waiting for NavigationClient to be ready...")
        # TODO: wait for nav2 to be ready
        # self.nav.waitUntilNav2Active()
        logger.info("NavigationClient is ready.")

    def _get_result_callback(self, future):
        logger.debug(f"NavigationClient received result. {future.result()}")

    def _goal_response_callback(self, future):
        logger.debug("NavigationClient received goal response.")
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            logger.error("Goal rejected")
            self.status = NavigationState.REJECTED
            return
        logger.info("Goal accepted")
        self.status = NavigationState.ACTIVE
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def send_goal(self, position_x, position_y, orientation):
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            logger.debug("NavigationClient is not ready yet, retrying...")

        self.last_dr = 0.0
        self.last_t0 = 0.0
        self.last_doz = 0.0
        self.count_lost = 0
        self.goal_position_x = position_x
        self.goal_position_y = position_y
        self.goal_orientation = orientation

        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, orientation)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""

        logger.info(
            "Navigating to x: {:0.2f}, y: {:0.2f} ...".format(
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y
            )
        )

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _cancel_response_callback(self, future):
        logger.info("NavigationClient received cancel response.")

    def cancel_goal(self):
        if self.result_future:
            logger.info("Canceling goal...")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_response_callback)

    def cancel_all_goals(self):
        logger.info("Canceling all goals...")
        self.cancel_goal()

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return False  # before was True
        if self.result_future.result():
            status = self.result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.status = NavigationState.SUCCEEDED
            elif status == GoalStatus.STATUS_ABORTED:
                self.status = NavigationState.FAILED
            elif status == GoalStatus.STATUS_CANCELED:
                self.status = NavigationState.CANCELED
            else:
                self.status = NavigationState.UNKNOWN
            return True
        else:
            # Timed out, still processing, not complete yet
            return False

    def wait_until_task_complete(self):
        logger.info("Waiting for NavigationClient to receive goal response...")
        while self.status == NavigationState.READY:
            # rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        if self.status == NavigationState.REJECTED:
            logger.warning("Goal was rejected")
            return self.status

        logger.info("Start wait to complete task")

        while not self.isTaskComplete():
            time.sleep(0.1)

        self.goal_handle = None
        self.result_future = None

        return self.status

    def get_result(self):
        return self.status


# ----- Example of use (after changing to NAVIGATION mode) -------
# $ python3 navstack_client.py --waypoint 2.0,0.0,1.57
if __name__ == "__main__":
    import threading
    import argparse

    parser = argparse.ArgumentParser(description="NavStackClient")
    parser.add_argument("--waypoint", type=str, default="0.0,0.0,0.0", help="waypoint")
    args = parser.parse_args()
    waypoint = args.waypoint.split(",")
    waypoint = [float(x) for x in waypoint]
    # show loggers
    # logging.basicConfig(level=logging.DEBUG)
    rclpy.init()

    node = rclpy.create_node("navstack_client")
    thread = threading.Thread(target=rclpy.spin, args=[node])
    thread.daemon = True
    thread.start()

    nav = NavStackClient(node)

    def feedback_callback(feedback_msg):
        if (time.time() - nav.last_t0) > 1.0:
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
                (current_position_x - nav.goal_position_x) ** 2
                + (current_position_y - nav.goal_position_y) ** 2
            )
            delta_orientation = abs(current_orientation - nav.goal_orientation)

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
            delta_dr = (nav.last_dr - dr) / (nav.last_t0 - time.time())
            delta_doz = abs(nav.last_doz - delta_orientation)
            nav.last_dr = dr
            nav.last_doz = delta_orientation

            out_string = f"NavFb ETA:{eta:2.1f} s, "
            out_string += f"Dr:{dr:2.2f} m, "
            out_string += f"Deu:{euclidean_distance:2.2f} m, "
            out_string += f"doz: {delta_orientation:2.1f} rad, "
            out_string += f"Tt:{tt:2.1f} s, "
            out_string += f"R:{r},\t "
            out_string += f"V:{delta_dr:2.2f} m/s, "
            out_string += f"d_doz:{delta_doz:2.1f} rad/s"
            print(out_string)
            nav.last_t0 = time.time()
            if delta_dr >= 0.0 and delta_doz == 0.0:
                nav.count_lost += 1
            else:
                nav.count_lost = 0
            if nav.count_lost > 5:
                print("Robot not moving")

    nav.feedback_callback = feedback_callback
    nav.try_create_client()
    nav.wait_until_ready()

    # nav.send_goal(8.0, 1.0, 0.0)
    # nav.send_goal(4.0, 1.0, 0.0)

    nav.send_goal(waypoint[0], waypoint[1], waypoint[2])

    status = nav.wait_until_task_complete()

    print(f"NavigationClient result: {status}")
    node.destroy_node()
    try:
        rclpy.shutdown()
        print("rcply shutdown")
    except Exception:
        print("rcply already shutdown")
