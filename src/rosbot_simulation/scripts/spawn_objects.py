#!/usr/bin/env python3

import math
from pathlib import Path

import rospy
import xacro
import yaml
from gazebo_msgs.srv import SpawnModel


def _quat_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return math.sin(half), math.cos(half)


def _render_xacro(path, mappings):
    doc = xacro.process_file(path, mappings=mappings)
    return doc.toprettyxml(indent="  ")


class ObjectSpawner:
    def __init__(self):
        config_path = rospy.get_param("~objects_config")
        pkg_path = Path(rospy.get_param("~package_path"))
        self.world_ns = rospy.get_param("~world_namespace", "")

        with open(config_path, "r", encoding="utf-8") as fh:
            self.config = yaml.safe_load(fh) or {}

        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        self.puck_xacro = str(pkg_path / "urdf" / "puck.xacro")
        self.marker_xacro = str(pkg_path / "urdf" / "aruco_marker.xacro")

    def spawn_all(self):
        self._spawn_pucks(self.config.get("pucks", []))
        self._spawn_markers(self.config.get("drop_zones", []))

    def _spawn_pucks(self, pucks):
        color_map = {
            "red": {"r": "1.0", "g": "0.1", "b": "0.1"},
            "green": {"r": "0.1", "g": "0.9", "b": "0.1"},
            "blue": {"r": "0.2", "g": "0.2", "b": "1.0"},
        }
        for puck in pucks:
            name = puck["name"]
            color = puck.get("color", "red")
            rgb = color_map.get(color, color_map["red"])
            xml = _render_xacro(
                self.puck_xacro,
                mappings={"name": name, "r": rgb["r"], "g": rgb["g"], "b": rgb["b"]},
            )
            self._spawn_model(
                name=name,
                xml=xml,
                x=puck.get("x", 0.0),
                y=puck.get("y", 0.0),
                z=puck.get("z", 0.02),
                yaw=puck.get("yaw", 0.0),
            )

    def _spawn_markers(self, zones):
        for zone in zones:
            name = zone["name"]
            marker_id = int(zone.get("marker_id", 1))
            material_name = f"Aruco/Marker{marker_id}"
            xml = _render_xacro(
                self.marker_xacro,
                mappings={"name": name, "material_name": material_name},
            )
            self._spawn_model(
                name=name,
                xml=xml,
                x=zone.get("x", 0.0),
                y=zone.get("y", 0.0),
                z=zone.get("z", 0.15),
                yaw=zone.get("yaw", 0.0),
            )

    def _spawn_model(self, name, xml, x, y, z, yaw):
        qz, qw = _quat_from_yaw(yaw)
        pose = f"""
position:
  x: {float(x)}
  y: {float(y)}
  z: {float(z)}
orientation:
  x: 0.0
  y: 0.0
  z: {qz}
  w: {qw}
"""
        pose_obj = yaml.safe_load(pose)
        from geometry_msgs.msg import Pose

        p = Pose()
        p.position.x = pose_obj["position"]["x"]
        p.position.y = pose_obj["position"]["y"]
        p.position.z = pose_obj["position"]["z"]
        p.orientation.x = pose_obj["orientation"]["x"]
        p.orientation.y = pose_obj["orientation"]["y"]
        p.orientation.z = pose_obj["orientation"]["z"]
        p.orientation.w = pose_obj["orientation"]["w"]

        try:
            self.spawn_srv(name, xml, self.world_ns, p, "world")
            rospy.loginfo("[SIM] Spawned model: %s", name)
        except rospy.ServiceException as err:
            rospy.logwarn("[SIM] Failed to spawn %s: %s", name, str(err))


def main():
    rospy.init_node("spawn_objects")
    spawner = ObjectSpawner()
    rospy.sleep(1.0)
    spawner.spawn_all()


if __name__ == "__main__":
    main()
