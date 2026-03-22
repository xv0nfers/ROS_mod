#!/usr/bin/env python3

"""OpenCV-based deterministic feature tracker for USV guidance."""

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsvOpenCvTrackerNode(Node):
    """Track moving features in the water ROI and emit a guidance vector."""

    def __init__(self) -> None:
        """Initialize node parameters, ROS interfaces, and tracker state."""
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
        """Process each camera frame and publish guidance/debug outputs."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        image_h, image_w = gray.shape
        horizon_y = int(image_h * self.roi_top_ratio)
        roi_mask = self._build_roi_mask(gray.shape, horizon_y)

        if self._need_reseed():
            self._reseed_features(gray, roi_mask)
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        next_points, status = self._compute_optical_flow(gray)
        if next_points is None or status is None:
            self.prev_gray = gray
            self.prev_points = None
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        good_new, good_old = self._select_valid_points(next_points, status)
        if len(good_new) < self.min_track_points:
            self._reseed_features(gray, roi_mask)
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, None, None)
            return

        moving_points, moving_flow = self._extract_motion(good_new, good_old)
        lock_active = len(moving_points) >= self.min_track_points

        if lock_active:
            center = np.mean(moving_points, axis=0)
            mean_flow = np.mean(moving_flow, axis=0)
            azimuth_norm, range_proxy = self._compute_guidance(
                center,
                mean_flow,
                image_w,
            )
            self._publish_target(azimuth_norm, range_proxy, True)
            self._publish_debug(frame, horizon_y, moving_points, center)
        else:
            self._publish_target(0.0, 0.0, False)
            self._publish_debug(frame, horizon_y, moving_points, None)

        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2)

    def _need_reseed(self) -> bool:
        """Return True when there are not enough previous points to track."""
        return (
            self.prev_gray is None
            or self.prev_points is None
            or len(self.prev_points) < 8
        )

    def _build_roi_mask(self, shape: Tuple[int, int], horizon_y: int) -> np.ndarray:
        """Create a binary mask that keeps only the bottom water ROI."""
        mask = np.zeros(shape, dtype=np.uint8)
        mask[horizon_y:, :] = 255
        return mask

    def _reseed_features(self, gray: np.ndarray, mask: np.ndarray) -> None:
        """Reinitialize corner features in the ROI for the next frame."""
        self.prev_points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            mask=mask,
        )
        self.prev_gray = gray

    def _compute_optical_flow(
        self,
        gray: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Compute sparse LK optical flow for tracked feature points."""
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None,
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        return next_points, status

    def _select_valid_points(
        self,
        next_points: np.ndarray,
        status: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Filter optical-flow results by valid tracking status."""
        valid = status.flatten() == 1
        good_new = next_points[valid]
        good_old = self.prev_points[valid]
        return good_new, good_old

    def _extract_motion(
        self,
        good_new: np.ndarray,
        good_old: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Keep points whose motion magnitude passes a threshold."""
        flow = good_new - good_old
        magnitudes = np.linalg.norm(flow, axis=1)
        moving_mask = magnitudes >= self.flow_min_magnitude
        return good_new[moving_mask], flow[moving_mask]

    def _compute_guidance(
        self,
        center: np.ndarray,
        mean_flow: np.ndarray,
        image_w: int,
    ) -> Tuple[float, float]:
        """Convert visual motion to simple deterministic guidance values."""
        azimuth_px = float(center[0] - (image_w * 0.5))
        azimuth_norm = float(np.clip(azimuth_px / (image_w * 0.5), -1.0, 1.0))

        # Simple range proxy from flow magnitude.
        # Lower apparent flow (for similar own-ship speed) indicates farther targets.
        flow_mag = float(np.linalg.norm(mean_flow))
        range_proxy = 1.0 / max(flow_mag, 1e-3)

        return azimuth_norm, range_proxy

    def _publish_target(self, azimuth_norm: float, range_proxy: float, lock: bool) -> None:
        """Publish guidance vector and lock state in a Twist message."""
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
        """Publish visualization with ROI line, features, and lock center."""
        image_h, image_w = frame.shape[:2]
        dbg = frame.copy()

        cv2.line(dbg, (0, horizon_y), (image_w, horizon_y), (0, 255, 255), 2)
        cv2.line(dbg, (image_w // 2, 0), (image_w // 2, image_h), (255, 255, 0), 1)

        if points is not None:
            for point in points[:250]:
                x, y = int(point[0]), int(point[1])
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
    """Entry point for the ROS 2 OpenCV tracker node."""
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
