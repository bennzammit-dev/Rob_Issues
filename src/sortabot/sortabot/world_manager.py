#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# We'll try importing the common Gazebo/ros_gz service types dynamically.
# Different ROS/Gazebo releases expose slightly different names.
try:
    # preferred modern names (ros_gz_interfaces)
    from ros_gz_interfaces.srv import CreateEntity as SrvCreateEntity
    from ros_gz_interfaces.srv import RemoveEntity as SrvRemoveEntity
    CREATE_NAME = "CreateEntity"
    REMOVE_NAME = "RemoveEntity"
except Exception:
    try:
        # alternative names (ros_gz_interfaces)
        from ros_gz_interfaces.srv import SpawnEntity as SrvCreateEntity
        from ros_gz_interfaces.srv import DeleteEntity as SrvRemoveEntity
        CREATE_NAME = "SpawnEntity"
        REMOVE_NAME = "DeleteEntity"
    except Exception:
        try:
            # older ros_gz_sim_msgs
            from ros_gz_sim_msgs.srv import CreateEntity as SrvCreateEntity
            from ros_gz_sim_msgs.srv import RemoveEntity as SrvRemoveEntity
            CREATE_NAME = "ros_gz_sim_msgs.CreateEntity"
            REMOVE_NAME = "ros_gz_sim_msgs.RemoveEntity"
        except Exception:
            SrvCreateEntity = None
            SrvRemoveEntity = None
            CREATE_NAME = None
            REMOVE_NAME = None

from std_srvs.srv import Empty
from std_msgs.msg import String

import json
import math
import time

from sortabot.generate_world import generate

SAFE_RADIUS = 1.5


class WorldManager(Node):
    def __init__(self):
        super().__init__("world_manager")

        self.world_name = "sortabot_world"

        # list of candidate spawn/delete service names (try them in order)
        self._spawn_candidates = [
            f"/world/{self.world_name}/create",
            f"/world/{self.world_name}/create_multiple",
            f"/world/{self.world_name}/create_entity",
            "/spawn_entity",
            "/spawn_sdf",  # less common
        ]
        self._delete_candidates = [
            f"/world/{self.world_name}/remove",
            f"/world/{self.world_name}/delete",
            "/delete_entity",
            "/remove_entity",
        ]

        # Determine available service type and actual service name
        if SrvCreateEntity is None or SrvRemoveEntity is None:
            self.get_logger().error(
                "No known ros-gz service classes imported (Create/Spawn / Remove/Delete). "
                "Make sure ros_gz packages are installed for your ROS distro."
            )
            raise RuntimeError("Required ros_gz service types not available")

        # Try to find a working spawn service name for the imported service class
        self.spawn_cli, self.spawn_service_name = self._find_service_client(
            SrvCreateEntity, self._spawn_candidates, "spawn/create"
        )

        # Try to find a working delete service name for the imported delete service class
        self.delete_cli, self.delete_service_name = self._find_service_client(
            SrvRemoveEntity, self._delete_candidates, "delete/remove"
        )

        self.get_logger().info(
            f"Using spawn service '{self.spawn_service_name}' ({CREATE_NAME}), "
            f"delete service '{self.delete_service_name}' ({REMOVE_NAME})"
        )

        # Reset service
        self.reset_srv = self.create_service(Empty, "reset_task", self.reset_cb)

        # Task publisher
        self.task_pub = self.create_publisher(String, "task_definition", 10)

        # bookkeeping
        self.spawned_entities = []
        self.robot_pos = None
        self.child_positions = []

        # initial spawn
        self.spawn_task()

    # ---------------- helper: find service ----------------
    def _find_service_client(self, srv_type, candidate_names, label):
        """Try multiple service names for srv_type, return (client, chosen_name)."""
        self.get_logger().info(f"Probing for Gazebo '{label}' services (this may take a few seconds)...")
        for name in candidate_names:
            try:
                cli = self.create_client(srv_type, name)
                self.get_logger().debug(f"Probing {name} ...")
                if cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f"Found service {name} for {label}")
                    return cli, name
                else:
                    # destroy client and try next
                    self.destroy_client(cli)
            except Exception:
                # if create_client raised, ignore and try next
                pass
        # if not found, raise with diagnostics
        msg = f"Could not find a usable service for {label}; tried: {candidate_names}"
        self.get_logger().error(msg)
        raise RuntimeError(msg)

    def destroy_client(self, cli):
        try:
            # rclpy has no explicit destroy_client in older versions; safe to ignore
            if hasattr(self, "destroy_node"):
                # best-effort cleanup (if available)
                pass
        except Exception:
            pass

    # ---------------- Utility ----------------
    def dist(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def too_close(self, pos):
        if self.robot_pos and self.dist(pos, self.robot_pos) < SAFE_RADIUS:
            return True
        for c in self.child_positions:
            if self.dist(pos, c) < SAFE_RADIUS:
                return True
        return False

    # ---------------- Spawn logic ----------------
    def spawn_task(self):
        robot, bins, objects, obstacles, childabots, _ = generate()

        if self.robot_pos is None:
            self.robot_pos = robot
            self.child_positions = [c["pos"] for c in childabots]

        objects = [o for o in objects if not self.too_close(o["pos"])]
        bins = [b for b in bins if not self.too_close(b["pos"])]

        # remove previously spawned (if any)
        self.spawned_entities.clear()

        for i, o in enumerate(objects):
            self._spawn_box_named(f"object_{i}", o["pos"], o["color"])

        for i, b in enumerate(bins):
            self._spawn_box_named(f"bin_{i}", b["pos"], b["color"], size=(1.0, 1.0, 0.8))

        task = {
            "objects": [
                {"id": f"object_{i}", "color": o["color"], "weight": o["weight"]}
                for i, o in enumerate(objects)
            ]
        }

        msg = String()
        msg.data = json.dumps(task)
        self.task_pub.publish(msg)

        self.get_logger().info("Task spawned")

    # ---------------- Spawn helpers ----------------
    def _spawn_box_named(self, name, pos, color, size=(0.5, 0.5, 0.4)):
        sx, sy, sz = size
        sdf = f"""<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{pos[0]} {pos[1]} {sz/2.0} 0 0 0</pose>
    <link name='link'>
      <collision name='collision'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{self.color_rgba(color)}</ambient>
          <diffuse>{self.color_rgba(color)}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

        # Construct request for whichever Create/Spawn type is available
        req = self._make_spawn_request(name, sdf)
        try:
            fut = self.spawn_cli.call_async(req)
            # do not block waiting for response; add to bookkeeping
            self.spawned_entities.append(name)
            self.get_logger().info(f"Requested spawn of '{name}' at {pos} via {self.spawn_service_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to call spawn service for {name}: {e}")

    def _make_spawn_request(self, name, xml):
        """Make and populate a request object for the detected spawn service type."""
        ReqType = type(self.spawn_cli._service_type).Request if hasattr(self.spawn_cli, "_service_type") else None

        # Try to construct a request object generically
        # We'll attempt to instantiate the Request class of the client type.
        try:
            req = self.spawn_cli._service_type.Request()  # best-effort (internal), sometimes present
        except Exception:
            # fallback: create Request from imported service class
            try:
                req = SrvCreateEntity.Request()
            except Exception:
                # very unlikely, but create a simple dummy object and set attributes dynamically
                class _DummyReq:
                    pass
                req = _DummyReq()

        # set common fields if available
        if hasattr(req, "name"):
            try:
                req.name = name
            except Exception:
                pass
        elif hasattr(req, "entity_name"):
            try:
                req.entity_name = name
            except Exception:
                pass

        if hasattr(req, "xml"):
            req.xml = xml
        elif hasattr(req, "sdf"):
            req.sdf = xml
        elif hasattr(req, "urdf"):
            req.urdf = xml
        # optional fields
        if hasattr(req, "allow_renaming"):
            req.allow_renaming = False
        if hasattr(req, "robot_namespace"):
            req.robot_namespace = ""
        if hasattr(req, "reference_frame"):
            req.reference_frame = ""
        return req

    # ---------------- Reset ----------------
    def reset_cb(self, req, res):
        self.get_logger().info("Resetting task (removing spawned entities)")
        for name in list(self.spawned_entities):
            try:
                rem_req = self._make_delete_request(name)
                self.delete_cli.call_async(rem_req)
            except Exception as e:
                self.get_logger().warn(f"Failed to request delete for {name}: {e}")
        time.sleep(0.2)
        self.spawn_task()
        return res

    def _make_delete_request(self, name):
        # Prefer Request() from the service class if available
        try:
            r = self.delete_cli._service_type.Request()  # best-effort
        except Exception:
            try:
                r = SrvRemoveEntity.Request()
            except Exception:
                class _D:
                    pass
                r = _D()
        # common field names
        if hasattr(r, "name"):
            r.name = name
        elif hasattr(r, "entity_name"):
            r.entity_name = name
        elif hasattr(r, "id"):
            r.id = name
        return r

    # ---------------- Colors ----------------
    def color_rgba(self, color):
        return {
            "red": "1 0 0 1",
            "green": "0 1 0 1",
            "blue": "0 0 1 1",
            "yellow": "1 1 0 1",
            "purple": "0.6 0 0.6 1",
            "white": "1 1 1 1",
            "black": "0 0 0 1"
        }.get(color, "1 1 1 1")


def main():
    rclpy.init()
    node = None
    try:
        node = WorldManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # print a full error so you can paste it if needed
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
