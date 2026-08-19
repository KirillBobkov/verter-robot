import os
import time
import uuid

from flask import Flask, send_file
from flask_restx import Api, Namespace, Resource, fields, reqparse


def create_app(shared_state, command_queue, result_dict, event_queue, map_image_path,):
    app = Flask(__name__)

    api = Api(
        app,
        version="1.0",
        title="Verter Robot API",
        description="Verter ROS 2 API",
        doc="/docs",
    )

    # -----------------------------------------
    # namespaces
    # -----------------------------------------

    waypoints_ns = Namespace(
        "waypoints",
        path="/waypoints",
    )

    waypoint_ns = Namespace(
        "waypoint",
        path="/waypoint",
    )

    position_ns = Namespace(
        "position",
        path="/position",
    )

    map_ns = Namespace(
        "map",
        path="/map",
    )

    api.add_namespace(waypoints_ns)
    api.add_namespace(waypoint_ns)
    api.add_namespace(position_ns)
    api.add_namespace(map_ns)
    
    pose_model = api.model(
        "Pose",
        {
            "x": fields.Float,
            "y": fields.Float,
            "yaw": fields.Float,
        },
    )

    waypoint_create_model = api.model(
        "WaypointCreate",
        {
            "id": fields.String(
                required=True,
                example="kitchen",
            ),

            "name": fields.String(
                required=True,
                example="Кухня",
            ),

            "is_base": fields.Boolean(
                default=False,
            ),

            "x": fields.Float(
                description=(
                    "Если отсутствует — "
                    "используется текущая позиция"
                ),
            ),

            "y": fields.Float,

            "yaw": fields.Float,
        },
    )

    waypoint_update_model = api.model(
        "WaypointUpdate",
        {
            "id": fields.String(
                required=True
            ),

            "name": fields.String,

            "is_base": fields.Boolean,

            "x": fields.Float,
            "y": fields.Float,
            "yaw": fields.Float,

            "use_current_position":
                fields.Boolean(
                    default=False
                ),
        },
    )

    waypoint_id_model = api.model(
        "WaypointId",
        {
            "id": fields.String(
                required=True,
                example="kitchen",
            )
        },
    )

    map_save_model = api.model(
        "MapSave",
        {
            "path": fields.String(
                example=(
                    "/home/jetson/maps/"
                    "current_map"
                )
            )
        },
    )
    
    def execute_ros_command(command, payload=None, timeout=3.0):
        request_id = uuid.uuid4().hex

        command_queue.put({
            "request_id": request_id,
            "command": command,
            "payload": payload or {}
        })

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            response = result_dict.pop(request_id, None)

            if response is not None:
                if response["success"]:
                    return response["result"], 200

                return {"error": response["error"]}, 400

            time.sleep(0.01)

        return {"error": "ROS manager timeout"}, 504
    
    @waypoints_ns.route("/list")
    class WaypointList(Resource):

        def get(self):
            waypoints = dict(shared_state.get("waypoints", {}))

            result = []

            for waypoint_id, waypoint in waypoints.items():
                result.append({"id": waypoint_id, **waypoint})

            return {"waypoints": result}, 200
    
    @waypoint_ns.route("/create")
    class WaypointCreate(Resource):

        @waypoint_ns.expect(waypoint_create_model, validate=True)
        def post(self):
            return execute_ros_command("waypoint_create", api.payload)
    
    @waypoint_ns.route("/update")
    class WaypointUpdate(Resource):

        @waypoint_ns.expect(waypoint_update_model, validate=True)
        def post(self):
            return execute_ros_command("waypoint_update", api.payload)
    
    @waypoint_ns.route("/delete")
    class WaypointDelete(Resource):

        @waypoint_ns.expect(
            waypoint_id_model,
            validate=True,
        )
        def post(self):

            return execute_ros_command(
                "waypoint_delete",
                api.payload,
            )
    
    @waypoint_ns.route("/goal")
    class WaypointGoal(Resource):

        @waypoint_ns.expect(waypoint_id_model, validate=True)
        def post(self):
            return execute_ros_command("waypoint_goal", api.payload)
    
    @position_ns.route("/current")
    class CurrentPosition(Resource):

        def get(self):
            return {
                "state": shared_state.get("state"),
                "localized": shared_state.get("localized", False),
                "position": shared_state.get("position"),
                "covariance": shared_state.get("covariance"),
            }, 200
    
    @map_ns.route("/save")
    class MapSave(Resource):

        @map_ns.expect(map_save_model, validate=False)
        def post(self):
            return execute_ros_command("map_save", api.payload or {})
    
    @map_ns.route("/show")
    class MapShow(Resource):

        @map_ns.produces(["image/png"])
        def get(self):
            if not os.path.exists(map_image_path):
                return {"error": "Map not available"}, 404

            return send_file(map_image_path, mimetype="image/png", max_age=0)
    
    @api.route("/events")
    class EventStream(Resource):

        @api.doc(produces=["text/event-stream"])
        def get(self):
            return {"message": "SSE will be implemented here"}, 501
    
    return app

def run_api(shared_state, command_queue, result_dict, event_queue, map_image_path,):
    app = create_app(shared_state, command_queue, result_dict, event_queue, map_image_path)
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False, use_reloader=False)