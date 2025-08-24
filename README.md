
# Delivery_bridge

This package is a bridge between ROS 2 ecosystem and a REST API.

This ROS 2 package have a node that start a HTTP server that provides a REST API, also start a Socket.IO server to provide a real-time communication between the web interface and the ROS 2 environment.


Tech Stack:

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [Python SocketIO 5.7.2](https://python-socketio.readthedocs.io/en/latest/)
- [FastAPI 0.105.0](https://fastapi.tiangolo.com/)


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

