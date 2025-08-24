
import time
from copy import deepcopy

from navigation_server.base_node import base_node
from navigation_server.webapp.socket_io import emitEvent


def run_on_moving_supervisor():
    base_node.logger.info(">" * 50)
    base_node.logger.info("Start on moving supervisor")
    last_pose = deepcopy(base_node.pose_subscriber.pose_data)
    last_cmd_vel = deepcopy(base_node.cmd_vel_publisher.pub_data)
    t0_last = time.time()
    on_moving = False
    while True:
        time.sleep(1)
        pose = base_node.pose_subscriber.pose_data
        cmd_vel = base_node.cmd_vel_publisher.pub_data

        # verify if robot is moving
        on_mov = abs(round((last_pose.position_x - pose.position_x), 2)) > 0.01
        on_mov |= abs(round((last_pose.position_y - pose.position_y), 2)) > 0.01
        on_mov |= abs(round((last_pose.orientation - pose.orientation), 2)) > 0.01

        # verify if user is sending commands
        on_mov |= abs(round((last_cmd_vel.linear.x - cmd_vel.linear.x), 2)) > 0.01
        on_mov |= abs(round((last_cmd_vel.angular.z - cmd_vel.angular.z), 2)) > 0.01

        if not on_moving and on_mov:
            t0_last = time.time()
            on_moving = True
            base_node.logger.info("Robot is moving")
            emitEvent("on_moving", {"on_moving": True})

        elif on_moving and on_mov:
            t0_last = time.time()

        elif on_moving and (time.time() - t0_last) > 60 * 5:
            on_moving = False
            base_node.logger.info("Robot stopped moving")
            emitEvent("on_moving", {"on_moving": False})

        last_pose = deepcopy(pose)
        last_cmd_vel = deepcopy(cmd_vel)
