"""WebServerNode — plain Node serving the static waypoint UI.

Justification for plain Node (not LifecycleNode):
  SI-9: This node is HMI-only. It has no role in the actuator command path.
  If this node crashes, patrol continues unaffected (verified by E2E test SI-9).
  The lifecycle overhead adds no safety value for a stateless HTTP file server.

The HTTP server runs in a daemon thread so it does not block the ROS executor.
PORT and HTML directory are configurable via ROS parameters.
"""
from __future__ import annotations

import functools
import http.server
import threading

import rclpy
from rclpy.node import Node


class WebServerNode(Node):
    """Serves web/index.html via stdlib HTTPServer in a daemon thread.

    Parameters
    ----------
    port : int
        TCP port for the HTTP server (default: 8080).
    html_dir : str
        Directory containing index.html. Defaults to the installed share/verter_admin/web.
    """

    def __init__(self) -> None:
        super().__init__('web_server_node')
        self.declare_parameter('port', 8080)

        # Default: installed share directory (set via launch argument for dev)
        try:
            from ament_index_python.packages import get_package_share_directory
            default_html_dir = get_package_share_directory('verter_admin') + '/web'
        except Exception:
            import os
            default_html_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..')

        self.declare_parameter('html_dir', default_html_dir)

        port = self.get_parameter('port').value
        html_dir = self.get_parameter('html_dir').value

        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler,
            directory=html_dir,
        )
        self._server = http.server.HTTPServer(('', port), handler)

        thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name='web_server_thread',
        )
        thread.start()
        self.get_logger().info(f'Web server listening on port {port}, serving {html_dir}')

    def destroy_node(self) -> None:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
