#!/usr/bin/env python3
"""
Web Server Node - HTTP-сервер для раздачи статического веб-интерфейса.

Раздаёт index.html на порту 8080 через http.server из stdlib.
Без внешних зависимостей.

Использование:
    ros2 run verter_admin web_server_node
    ros2 run verter_admin web_server_node --ros-args -p port:=8080
"""

import os
import threading
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')

        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Директория со статикой — share/verter_admin/web/
        pkg_share = get_package_share_directory('verter_admin')
        web_dir = os.path.join(pkg_share, 'web')

        if not os.path.isdir(web_dir):
            self.get_logger().error(f'Web-директория не найдена: {web_dir}')
            return

        handler = partial(SimpleHTTPRequestHandler, directory=web_dir)
        self._server = HTTPServer(('0.0.0.0', port), handler)

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Web-сервер запущен на порту {port}, директория: {web_dir}')

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
