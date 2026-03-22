#!/usr/bin/env python3

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsvOpenCvTrackerNode(Node):
    """Deterministic OpenCV feature-tracking node for USV guidance."""

    def __init__(self) -> None:
        super().__init__("usv_opencv_tracker_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("target_vector_topic", "/usv/target_vector")
        self.declare_parameter("debug_image_topic", "/usv/vision_debug")
        self.declare_parameter("roi_top_ratio", 0.30)
        self.declare_parameter("max_corners", 120)
        self.declare_parameter("quality_level", 0.01)
        self.declare_parameter("min_distance", 7.0)
        self.declare_parameter("min_track_points", 15)
        self.declare_parameter("flow_min_magnitude", 1.5)

        self.image_topic = self.get_parameter("image_topic").value
        self.target_vector_topic = self.get_parameter("target_vector_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.roi_top_ratio = float(self.get_parameter("roi_top_ratio").value)
        self.max_corners = int(self.get_parameter("max_corners").value)
        self.quality_level = float(self.get_parameter("quality_level").value)
        self.min_distance = float(self.get_parameter("min_distance").value)
        self.min_track_points = int(self.get_parameter("min_track_points").value)
        self.flow_min_magnitude = float(self.get_parameter("flow_min_magnitude").value)

        self.bridge = CvBridge()
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_points: Optional[np.ndarray] = None

        self.target_pub = self.create_publisher(Twist, self.target_vector_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info("USV OpenCV tracker node started")

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape
        horizon_y = int(h * self.roi_top_ratio)

        # Mask top area to ignore sky/clouds and keep marine ROI.
        roi_mask = np.zeros_like(gray, dtype=np.uint8)
        roi_mask[horizon_y:, :] = 255

        if self.prev_gray is None or self.prev_points is None or len(self.prev_points) < 8:
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_corners,
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
                mask=roi_mask,
            )
            self.prev_gray = gray
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None,
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        if next_points is None or status is None:
            self.prev_gray = gray
            self.prev_points = None
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        good_new = next_points[status.flatten() == 1]
        good_old = self.prev_points[status.flatten() == 1]

        if len(good_new) < self.min_track_points:
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_corners,
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
                mask=roi_mask,
            )
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        flow = good_new - good_old
        magnitudes = np.linalg.norm(flow, axis=1)
        moving_mask = magnitudes >= self.flow_min_magnitude
        moving_points = good_new[moving_mask]
        moving_flow = flow[moving_mask]

        lock_active = len(moving_points) >= self.min_track_points

        if lock_active:
            center = np.mean(moving_points, axis=0)
            mean_flow = np.mean(moving_flow, axis=0)

            azimuth_px = float(center[0] - (w * 0.5))
            azimuth_norm = float(np.clip(azimuth_px / (w * 0.5), -1.0, 1.0))

            # Heuristic range proxy: weaker motion at same platform speed often means farther target.
            # Inverse of mean flow magnitude keeps behavior deterministic and lightweight.
            flow_mag = float(np.linalg.norm(mean_flow))
            range_proxy = 1.0 / max(flow_mag, 1e-3)

            self._publish_target(azimuth_norm, range_proxy, True)
            self._publish_debug(frame, horizon_y, moving_points, center)
        else:
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, moving_points, None)

        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2)

    def _publish_target(self, azimuth_norm: float, range_proxy: float, lock: bool) -> None:
        out = Twist()
        out.linear.x = float(azimuth_norm)
        out.linear.y = float(range_proxy)
        out.linear.z = 1.0 if lock else 0.0
        self.target_pub.publish(out)

    def _publish_debug(
        self,
        frame: np.ndarray,
        horizon_y: int,
        points: Optional[np.ndarray],
        center: Optional[np.ndarray],
    ) -> None:
        h, w = frame.shape[:2]
        dbg = frame.copy()

        cv2.line(dbg, (0, horizon_y), (w, horizon_y), (0, 255, 255), 2)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

        if points is not None:
            for p in points[:250]:
                x, y = int(p[0]), int(p[1])
                cv2.circle(dbg, (x, y), 2, (0, 255, 0), -1)

        if center is not None:
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(dbg, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(
                dbg,
                "LOCK(OPENCV)",
                (max(5, cx - 40), max(20, cy - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
        self.debug_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsvOpenCvTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
