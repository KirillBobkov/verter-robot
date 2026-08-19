import multiprocessing
import os

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from api import run_api
from manager_node import ManagerNode


def main(args=None):

    multiprocessing.set_start_method(
        "spawn",
        force=True,
    )

    manager = multiprocessing.Manager()

    shared_state = manager.dict()
    result_dict = manager.dict()

    command_queue = (
        multiprocessing.Queue()
    )

    event_queue = (
        multiprocessing.Queue()
    )

    #package_dir = (
    #    get_package_share_directory(
    #        "verter_manager"
    #    )
    #)

    waypoints_path = os.path.join(
        #package_dir,
        "/home/jetson/verter-robot/verter_admin/src/verter_admin/"
        "config",
        "waypoints.yaml",
    )

    map_image_path = (
        "/tmp/verter_map.png"
    )

    # ------------------------------------
    # Flask
    # ------------------------------------

    api_process = (
        multiprocessing.Process(
            target=run_api,
            kwargs={
                "shared_state": shared_state,
                "command_queue": command_queue,
                "result_dict": result_dict,
                "event_queue": event_queue,
                "map_image_path": map_image_path,
            },
            name="verter_api",
        )
    )

    api_process.start()

    # ------------------------------------
    # ROS
    # ------------------------------------

    rclpy.init(args=args)

    node = ManagerNode(
        shared_state=shared_state,
        command_queue=command_queue,
        event_queue=event_queue,
        result_dict=result_dict,
        waypoints_path=waypoints_path,
        map_image_path=map_image_path,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        api_process.terminate()
        api_process.join(timeout=3.0)

        manager.shutdown()


if __name__ == "__main__":
    main()