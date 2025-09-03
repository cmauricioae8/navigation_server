
# NAVIGATION_SERVER

This package is a bridge between ROS 2 ecosystem and a REST API.

This ROS 2 package have a node that start a HTTP server that provides a REST API, also start a Socket.IO server to provide a real-time communication between the web interface and the ROS 2 environment.


Tech Stack:

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [Python SocketIO 5.7.2](https://python-socketio.readthedocs.io/en/latest/)
- [FastAPI 0.105.0](https://fastapi.tiangolo.com/)

## Pre-requisites

Since this package is used as the back-end in robots of Octopy, to test correctly all functionalities that this repo provides, some configurations must be made:

* At least packages 'core_interfaces', 'robot_bringup' and 'robot_core' must be built.
* Environment variables both 'ROBOT_MODEL', 'LIDAR_MODEL' and 'WITH_DEEP_CAM' must be added in the '.bashrc' file.
* '~/.robot_config' folder with the corresponding files must exist, including the **'navigation_server/server_node.yaml'**. Note: if this file does not exist, run the server_node as a node, do not use the launcher file.


## Installation

In order to use this package, you need to have ROS2 installed in your machine. You can follow the instructions in the [ROS2 official website](https://docs.ros.org/en/humble/Installation.html).

After you have ROS 2 installed, you can clone this repository in your workspace and build the package.

```bash
cd ~/colcon_ws/src
git clone https://github.com/cmauricioae8/navigation_server.git
cd navigation_server
pip install -r requirements.txt
cd ~/colcon_ws/
colcon build --packages-select navigation_server --symlink-install --allow-overriding navigation_server
source install/setup.bash
```

After of above configurations, test the project:

```bash
ros2 run navigation_server server_node
```

Go to local url http://127.0.0.1:9009 and check that all is going well

