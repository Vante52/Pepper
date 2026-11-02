"""Inicia puente HTTP y cliente ROS 2 sin mezclar ejecutores/hilos."""

import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from .catalog import BackendRouter, load_catalog
from .http_api import create_server
from .trajectory_backend import TrajectoryBackend
from .trajectory_profile import parse_trajectory


class SkillBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_skill_bridge')
        default_path = str(Path(get_package_share_directory('robot_skill_bridge'))
                           / 'config' / 'pepper_skills.yaml')
        self.declare_parameter('config_file', default_path)
        self.declare_parameter('bind_host', '127.0.0.1')
        self.declare_parameter('port', 8765)


def main(args=None):
    rclpy.init(args=args)
    node = None
    server = None
    try:
        node = SkillBridgeNode()
        catalog = load_catalog(node.get_parameter('config_file').value, {
            'follow_joint_trajectory': parse_trajectory,
        })
        host = node.get_parameter('bind_host').value
        port = node.get_parameter('port').value
        if not isinstance(host, str) or not isinstance(port, int) or not 0 < port < 65536:
            raise ValueError('bind_host o port inválido')
        if host not in ('127.0.0.1', 'localhost'):
            raise ValueError('MVP local: bind_host debe ser loopback (sin autenticación)')
        backend = BackendRouter({'follow_joint_trajectory': TrajectoryBackend(node)})
        server = create_server(host, port, catalog, backend)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f'API HTTP activa: http://{host}:{port}/v1/skills')
        node.get_logger().info(f'Habilidades permitidas: {", ".join(catalog)}')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
