#!/usr/bin/env python3

import math

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft

# Importing from tf2_geometry_msgs registers PointStamped/PoseStamped
# converters with tf2_ros as an import side-effect.
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from rosbot_competition_msgs.msg import SpatialDetection, MissionEvent
from sensor_msgs.msg import CameraInfo, Image
from image_geometry import PinholeCameraModel


class PerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.1))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 2.5))
        self.min_contour_area = float(rospy.get_param("~min_contour_area", 450.0))
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius_px", 2))
        self.publish_cooldown_sec = float(rospy.get_param("~publish_cooldown_sec", 0.4))
        self.min_publish_distance_m = float(
            rospy.get_param("~min_publish_distance_m", 0.05)
        )
        self.tf_lookup_timeout_sec = float(
            rospy.get_param("~tf_lookup_timeout_sec", 0.07)
        )
        # Remove noisy sky/ceiling band: CV + ArUco run on the image below this strip.
        self.crop_top_fraction = float(rospy.get_param("~crop_top_fraction", 0.20))

        self.hsv_ranges = rospy.get_param("~hsv_ranges", {})
        aruco_cfg = rospy.get_param("~aruco", {})
        self.marker_size_m = float(aruco_cfg.get("marker_size_m", 0.25))
        self.marker_color_map = {
            int(marker_id): color
            for marker_id, color in aruco_cfg.get(
                "marker_color_map", {1: "red", 2: "green", 3: "blue"}
            ).items()
        }

        self.object_tf_publish_rate_hz = float(
            rospy.get_param("~object_tf_publish_rate_hz", 12.0)
        )
        self.object_tf_max_age_sec = float(
            rospy.get_param("~object_tf_max_age_sec", 4.0)
        )

        dictionary_name = aruco_cfg.get("dictionary", "DICT_4X4_50")
        self.aruco, self.aruco_dict, self.aruco_detector = self._build_aruco_detector(
            dictionary_name
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None

        self.last_detection = {}
        self.object_frames = {}

        self.object_tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.det_pub = rospy.Publisher(
            "/perception/spatial_detections", SpatialDetection, queue_size=30
        )
        self.annotated_pub = rospy.Publisher(
            "/perception/annotated_image", Image, queue_size=1
        )
        self.event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=30)

        self.camera_model = PinholeCameraModel()
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1
        )

        self.color_sub = message_filters.Subscriber(self.color_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.08
        )
        self.sync.registerCallback(self._on_image_pair)

        period = max(1.0 / max(self.object_tf_publish_rate_hz, 1e-3), 0.05)
        self.tf_timer = rospy.Timer(
            rospy.Duration(period), self._broadcast_object_frames
        )

        rospy.loginfo("[VISION] Perception node online")

    def _build_aruco_detector(self, dictionary_name):
        if not hasattr(cv2, "aruco"):
            rospy.logwarn("[VISION] OpenCV ArUco module not available")
            return None, None, None

        aruco = cv2.aruco
        dict_id = getattr(aruco, dictionary_name, aruco.DICT_4X4_50)
        dictionary = aruco.getPredefinedDictionary(dict_id)

        if hasattr(aruco, "ArucoDetector") and hasattr(aruco, "DetectorParameters"):
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        else:
            detector = aruco.DetectorParameters_create()

        return aruco, dictionary, detector

    def _on_camera_info(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D, dtype=np.float64).reshape(-1, 1)
        if self.camera_info_sub is not None:
            self.camera_info_sub.unregister()
            self.camera_info_sub = None
            rospy.loginfo("[VISION] Camera intrinsics initialized")

    def _on_image_pair(self, color_msg, depth_msg):
        if self.fx is None:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as err:
            rospy.logwarn_throttle(
                2.0, "[VISION] CvBridge conversion failed: %s", str(err)
            )
            return

        depth_m = self._to_meters(depth)
        if depth_m is None:
            return

        h = bgr.shape[0]
        crop_top = min(h - 1, int(round(h * self.crop_top_fraction)))
        if crop_top > 0:
            bgr = bgr[crop_top:, :]
            depth_m = depth_m[crop_top:, :]

        overlay = bgr.copy()
        timestamp = color_msg.header.stamp
        camera_frame = (
            self.camera_frame_override
            if self.camera_frame_override
            else color_msg.header.frame_id
        )

        puck_candidates = self._detect_pucks(overlay)
        for candidate in puck_candidates:
            self._publish_spatial_detection(
                overlay=overlay,
                stamp=timestamp,
                camera_frame=camera_frame,
                depth_m=depth_m,
                object_class="puck",
                color=candidate["color"],
                marker_id=-1,
                u=candidate["u"],
                v=candidate["v"],
                crop_top_px=crop_top,
                confidence=candidate["confidence"],
            )

        drop_zone_candidates = self._detect_markers(overlay, crop_top)
        for candidate in drop_zone_candidates:
            self._publish_spatial_detection(
                overlay=overlay,
                stamp=timestamp,
                camera_frame=camera_frame,
                depth_m=depth_m,
                object_class="drop_zone",
                color=candidate["color"],
                marker_id=candidate["marker_id"],
                u=candidate["u"],
                v=candidate["v"],
                crop_top_px=crop_top,
                confidence=1.0,
                marker_corners_full=candidate["corners_full"],
            )

        try:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            )
        except CvBridgeError:
            pass

    def _to_meters(self, depth_image):
        if depth_image is None:
            return None

        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * self.depth_scale

        if depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
            return depth_image.astype(np.float32)

        rospy.logwarn_throttle(
            5.0, "[VISION] Unsupported depth dtype: %s", str(depth_image.dtype)
        )
        return None

    def _detect_pucks(self, overlay):
        hsv = cv2.cvtColor(overlay, cv2.COLOR_BGR2HSV)
        detections = []
        image_area = float(overlay.shape[0] * overlay.shape[1])

        # Always re-run HSV detection on every frame. The previous version
        # cached colours that had ever been published and skipped them, which
        # froze the puck position estimate at the very first (far, noisy)
        # detection and left the mission state machine driving toward stale
        # coordinates that no longer matched reality.
        for color_name, ranges in self.hsv_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for hsv_range in ranges:
                lower = np.array(hsv_range["lower"], dtype=np.uint8)
                upper = np.array(hsv_range["upper"], dtype=np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self.min_contour_area:
                continue

            moments = cv2.moments(largest)
            if moments["m00"] <= 0.0:
                continue

            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            confidence = min(1.0, area / max(image_area * 0.08, 1.0))

            cv2.drawContours(overlay, [largest], -1, (255, 255, 255), 2)
            cv2.circle(overlay, (u, v), 4, (0, 0, 0), -1)
            cv2.putText(
                overlay,
                f"{color_name} puck",
                (u + 6, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            detections.append(
                {"color": color_name, "u": u, "v": v, "confidence": confidence}
            )

        return detections

    def _detect_markers(self, overlay, crop_top_px):
        detections = []
        if self.aruco is None or self.aruco_dict is None:
            return detections

        gray = cv2.cvtColor(overlay, cv2.COLOR_BGR2GRAY)
        if hasattr(self.aruco, "ArucoDetector") and hasattr(
            self.aruco_detector, "detectMarkers"
        ):
            corners, ids, _rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = self.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_detector
            )

        if ids is None or len(ids) == 0:
            return detections

        self.aruco.drawDetectedMarkers(overlay, corners, ids)
        for idx, marker_id in enumerate(ids.flatten()):
            pts = corners[idx][0]
            pts_full = pts.copy()
            pts_full[:, 1] += float(crop_top_px)
            center = np.mean(pts, axis=0)
            u = int(center[0])
            v = int(center[1])
            color = self.marker_color_map.get(int(marker_id), "unknown")
            cv2.putText(
                overlay,
                f"ID {int(marker_id)} ({color})",
                (u + 8, v + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
            detections.append(
                {
                    "marker_id": int(marker_id),
                    "color": color,
                    "u": u,
                    "v": v,
                    "corners_full": pts_full.astype(np.float32),
                }
            )

        return detections

    def _publish_spatial_detection(
        self,
        overlay,
        stamp,
        camera_frame,
        depth_m,
        object_class,
        color,
        marker_id,
        u,
        v,
        crop_top_px,
        confidence,
        marker_corners_full=None,
    ):
        try:
            tf_timeout = rospy.Duration(self.tf_lookup_timeout_sec)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, camera_frame, stamp, tf_timeout
            )
        except tf2_ros.ExtrapolationException:
            # Intentional: do not fall back to Time(0), because latest-TF
            # projection while turning/skidding can mis-map puck/tag positions.
            rospy.logwarn_throttle(
                2.0,
                "[VISION] Camera/TF timestamps are out of sync; dropping projection for this frame.",
            )
            return
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[VISION] TF lookup failed (%s -> %s @ %.3f): %s",
                camera_frame,
                self.map_frame,
                stamp.to_sec(),
                str(exc),
            )
            return

        map_point = None
        map_quat = (0.0, 0.0, 0.0, 1.0)
        depth_value = None

        if object_class == "drop_zone" and marker_corners_full is not None:
            marker_pose = self._estimate_marker_pose(marker_corners_full)
            if marker_pose is not None:
                rvec, tvec = marker_pose
                map_point, map_quat = self._marker_pose_to_map(
                    stamp, camera_frame, transform, rvec, tvec
                )
                depth_value = float(np.linalg.norm(tvec))

        if map_point is None:
            z = self._sample_depth(depth_m, u, v)
            if z is None:
                return

            u_full = u
            v_full = v + int(crop_top_px)
            ray = self.camera_model.projectPixelTo3dRay((u_full, v_full))
            ray_z = ray[2]
            x = ray[0] * (z / ray_z)
            y = ray[1] * (z / ray_z)

            source = PointStamped()
            source.header.stamp = stamp
            source.header.frame_id = camera_frame
            source.point = Point(x=x, y=y, z=z)
            map_point = do_transform_point(source, transform).point
            depth_value = float(z)

        child_frame = self._object_frame_name(object_class, color, marker_id)
        if child_frame:
            self._upsert_object_frame(child_frame, map_point, map_quat, stamp)

        key = f"{object_class}:{color}:{marker_id}"
        if not self._should_publish(key, map_point, stamp):
            return

        msg = SpatialDetection()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.object_class = object_class
        msg.color = color
        msg.marker_id = marker_id
        msg.map_point = map_point
        msg.depth_m = float(depth_value)
        msg.confidence = float(confidence)
        self.det_pub.publish(msg)

        cv2.circle(overlay, (u, v), 7, (0, 255, 255), 2)

        event_msg = MissionEvent()
        event_msg.level = "INFO"
        event_msg.message = f"[VISION] Mapped {color} {object_class} at ({map_point.x:.2f}, {map_point.y:.2f})"
        self.event_pub.publish(event_msg)

    def _object_frame_name(self, object_class, color, marker_id):
        safe_color = str(color).strip().lower()
        if object_class == "puck" and safe_color:
            return f"puck_{safe_color}_frame"
        if object_class == "drop_zone":
            if safe_color and safe_color != "unknown":
                return f"drop_zone_{safe_color}_frame"
            if int(marker_id) >= 0:
                return f"drop_zone_marker_{int(marker_id)}_frame"
        return ""

    def _upsert_object_frame(self, child_frame, map_point, quat_xyzw, stamp):
        self.object_frames[child_frame] = {
            "stamp": stamp,
            "point": Point(
                x=float(map_point.x), y=float(map_point.y), z=float(map_point.z)
            ),
            "quat": (
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
                float(quat_xyzw[3]),
            ),
        }

    def _broadcast_object_frames(self, _event):
        now = rospy.Time.now()
        transforms = []
        stale = []
        for child_frame, data in list(self.object_frames.items()):
            if (
                self.object_tf_max_age_sec > 0.0
                and (now - data["stamp"]).to_sec() > self.object_tf_max_age_sec
            ):
                stale.append(child_frame)
                continue

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = child_frame
            tf_msg.transform.translation.x = data["point"].x
            tf_msg.transform.translation.y = data["point"].y
            tf_msg.transform.translation.z = data["point"].z
            qx, qy, qz, qw = data["quat"]
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            transforms.append(tf_msg)

        for child_frame in stale:
            self.object_frames.pop(child_frame, None)

        if transforms:
            self.object_tf_broadcaster.sendTransform(transforms)

    def _estimate_marker_pose(self, marker_corners_full):
        if (
            self.aruco is None
            or self.camera_matrix is None
            or self.dist_coeffs is None
            or self.marker_size_m <= 0.0
            or not hasattr(self.aruco, "estimatePoseSingleMarkers")
        ):
            return None

        corners = np.asarray(marker_corners_full, dtype=np.float32).reshape(1, 4, 2)
        result = self.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if result is None:
            return None

        if len(result) == 3:
            rvecs, tvecs, _obj_points = result
        elif len(result) == 2:
            rvecs, tvecs = result
        else:
            return None
        if rvecs is None or tvecs is None or len(rvecs) == 0 or len(tvecs) == 0:
            return None
        return (
            np.asarray(rvecs[0], dtype=np.float64).reshape(3, 1),
            np.asarray(tvecs[0], dtype=np.float64).reshape(3),
        )

    def _marker_pose_to_map(self, stamp, camera_frame, map_from_camera, rvec, tvec):
        marker_point = PointStamped()
        marker_point.header.stamp = stamp
        marker_point.header.frame_id = camera_frame
        marker_point.point = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
        map_point = do_transform_point(marker_point, map_from_camera).point

        rot_cam_marker, _ = cv2.Rodrigues(rvec)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot_cam_marker
        q_cam_marker = tft.quaternion_from_matrix(mat)

        q_map_camera = (
            map_from_camera.transform.rotation.x,
            map_from_camera.transform.rotation.y,
            map_from_camera.transform.rotation.z,
            map_from_camera.transform.rotation.w,
        )
        q_map_marker = np.asarray(
            tft.quaternion_multiply(q_map_camera, q_cam_marker), dtype=np.float64
        )
        norm = np.linalg.norm(q_map_marker)
        if norm > 1e-8:
            q_map_marker /= norm
        else:
            q_map_marker[:] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        return map_point, tuple(float(v) for v in q_map_marker)

    def _should_publish(self, key, current_point, stamp):
        previous = self.last_detection.get(key)
        if previous is None:
            self.last_detection[key] = (stamp, current_point)
            return True

        previous_stamp, previous_point = previous
        dt = (stamp - previous_stamp).to_sec()
        distance = math.hypot(
            current_point.x - previous_point.x, current_point.y - previous_point.y
        )
        if dt >= self.publish_cooldown_sec or distance >= self.min_publish_distance_m:
            self.last_detection[key] = (stamp, current_point)
            return True
        return False

    def _sample_depth(self, depth_m, u, v):
        height, width = depth_m.shape[:2]
        x0 = max(0, int(u) - self.depth_patch_radius)
        y0 = max(0, int(v) - self.depth_patch_radius)
        x1 = min(width - 1, int(u) + self.depth_patch_radius)
        y1 = min(height - 1, int(v) + self.depth_patch_radius)

        patch = depth_m[y0 : y1 + 1, x0 : x1 + 1].reshape(-1)
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch > self.min_depth_m) & (patch < self.max_depth_m)]
        if patch.size == 0:
            return None
        return float(np.median(patch))


def main():
    rospy.init_node("perception_node")
    PerceptionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
