import rclpy
import uvicorn
from threading import Thread

from navigation_server.base_node import base_node
from navigation_server.webapp.main import app as webapp
from navigation_server.webapp.main import sio
from navigation_server.modules.mode_manager import mode_manager
from navigation_server.modules.on_moving_supervisor import run_on_moving_supervisor

from navigation_server.webapp.database import createDatabase
from navigation_server.webapp.apps.users.cruds.user_cruds import user_crud
from navigation_server.webapp.apps.users.models import User


def main(args=None):
    ################################################################################
    # Create the database if it doesn't exist
    db_success = createDatabase()
    if db_success:
        base_node.logger.info("Database created successfully.")
    else:
        base_node.logger.error("Database creation failed.")

    # Create the default users if they don't exist
    default_users = [
        {"name": "admin", "password":"admin"}, {"name": "administrador", "password":"0ct0p7"},
        {"name": "eventos", "password":"eventos"},
    ]
    
    for m_user in default_users:
        users: list[User] = user_crud.get_by_field("username", m_user['name'], allows_multiple=True)

        if len(users) == 0:
            base_node.logger.info(f"No {m_user['name']} user found, creating by default")
            user = User(username=m_user['name'], is_admin=True, is_active=True)
            user.set_password(m_user['password'])
            user.save()
            base_node.logger.info(f"{m_user['name']} user created")
    

    # Start the webapp
    webapp_thread = Thread(
        target=uvicorn.run, kwargs={"app": webapp, "host": "0.0.0.0", "port": 9009}
    )
    webapp_thread.daemon = True
    webapp_thread.start()


    # start on moving supervisor
    on_moving_supervisor_thread = Thread(target=run_on_moving_supervisor)
    on_moving_supervisor_thread.daemon = True
    on_moving_supervisor_thread.start()


    base_node.init_topics()
    mode_manager.start()

    # register sio events
    @sio.event
    async def cmd_vel(sid, data):
        linear_x = float(data["linear_x"])
        angular_z = float(data["angular_z"])
        # base_node.logger.info("robot_move \tx:{:2.2f}, \tz:{:2.2f}".format(linear_x, angular_z ))
        base_node.cmd_vel_publisher.publish(linear_x, angular_z)
    

    try:
        rclpy.spin(base_node)
    except KeyboardInterrupt:
        print("Ctrl - C detected")
        mode_manager.processes_stop()
    finally:
        base_node.destroy_node()


if __name__ == "__main__":
    main()
