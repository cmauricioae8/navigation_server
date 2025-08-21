import rclpy

from navigation_server.base_node import base_node


def main(args=None):
    ################################################################################

    base_node.init_topics()
    rclpy.spin(base_node)

    base_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
