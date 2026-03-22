#!/usr/bin/env python3

import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray


class UsvTargetSelectorNode(Node):
    """Bridge node between perception detections and USV guidance commands."""

    def __init__(self) -> None:
        super().__init__("usv_target_selector_node")

        # Parameters tuned for high-rate, low-latency edge execution.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/tracking")
        self.declare_parameter("target_vector_topic", "/usv/target_vector")
        self.declare_parameter("debug_image_topic", "/usv/vision_debug")
        self.declare_parameter("roi_top_ratio", 0.30)
        self.declare_parameter("priority_class_ids", [0, 1])
        self.declare_parameter("min_stable_hits", 3)
        self.declare_parameter("max_lock_misses", 5)
        self.declare_parameter("max_detection_age_sec", 0.25)

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.target_vector_topic = self.get_parameter("target_vector_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.roi_top_ratio = float(self.get_parameter("roi_top_ratio").value)
        self.priority_class_ids = {
            int(v) for v in self.get_parameter("priority_class_ids").value
        }
        self.min_stable_hits = int(self.get_parameter("min_stable_hits").value)
        self.max_lock_misses = int(self.get_parameter("max_lock_misses").value)
        self.max_detection_age_sec = float(
            self.get_parameter("max_detection_age_sec").value
        )

        self.cv_bridge = CvBridge()
        self.latest_detections: Optional[Detection2DArray] = None

        # Lock state machine.
        self.locked_track_id: Optional[str] = None
        self.lock_miss_count = 0
        self.track_hit_counter: Dict[str, int] = {}

        self.vector_pub = self.create_publisher(Twist, self.target_vector_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info("USV target selector node started.")

    def detections_callback(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg

    def image_callback(self, img_msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        frame_h, frame_w = cv_image.shape[:2]
        horizon_y = int(frame_h * self.roi_top_ratio)

        detections_msg = self.latest_detections
        detections = []
        if detections_msg is not None and self._is_detection_fresh(img_msg, detections_msg):
            detections = detections_msg.detections

        candidates = self._extract_candidates(detections, frame_w, frame_h, horizon_y)
        locked_candidate = self._update_lock_state(candidates)

        azimuth_norm = 0.0
        estimated_distance = 0.0
        lock_active = locked_candidate is not None

        if lock_active:
            cx, cy, bw, bh = locked_candidate["bbox"]

            # Guidance math:
            # pixel_error = target_center_x - optical_center_x
            # normalized_error in [-1, 1] = pixel_error / (image_width / 2)
            # This normalized value can be used directly by heading controller.
            azimuth_px = cx - (frame_w * 0.5)
            azimuth_norm = float(np.clip(azimuth_px / (frame_w * 0.5), -1.0, 1.0))

            # Distance heuristic (monocular): larger box means closer object.
            # Use size ratio as inverse-distance proxy and invert for a distance-like value.
            size_ratio = max(bw / max(frame_w, 1.0), bh / max(frame_h, 1.0))
            estimated_distance = 1.0 / max(size_ratio, 1e-3)

            self._draw_locked_target(
                cv_image,
                locked_candidate,
                azimuth_norm,
                horizon_y,
            )
        else:
            cv2.line(cv_image, (0, horizon_y), (frame_w, horizon_y), (0, 255, 255), 2)

        self._publish_target_vector(azimuth_norm, estimated_distance, lock_active)

        debug_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        debug_msg.header = img_msg.header
        self.debug_pub.publish(debug_msg)

    def _extract_candidates(
        self,
        detections: list,
        frame_w: int,
        frame_h: int,
        horizon_y: int,
    ) -> list:
        candidates = []

        for det in detections:
            track_id = self._extract_track_id(det)
            class_id, score = self._extract_class_and_score(det)
            if track_id is None or class_id is None:
                continue
            if class_id not in self.priority_class_ids:
                continue

            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            bw = float(det.bbox.size_x)
            bh = float(det.bbox.size_y)

            if bw <= 1.0 or bh <= 1.0:
                continue

            # Fast ROI filter: ignore top band (sky/clouds/birds) using bbox center.
            if cy < horizon_y:
                continue

            left = max(int(cx - bw * 0.5), 0)
            top = max(int(cy - bh * 0.5), 0)
            right = min(int(cx + bw * 0.5), frame_w - 1)
            bottom = min(int(cy + bh * 0.5), frame_h - 1)
            area = max(right - left, 0) * max(bottom - top, 0)

            candidates.append(
                {
                    "track_id": track_id,
                    "class_id": class_id,
                    "score": score,
                    "bbox": (cx, cy, bw, bh),
                    "rect": (left, top, right, bottom),
                    "area": area,
                }
            )

            self.track_hit_counter[track_id] = self.track_hit_counter.get(track_id, 0) + 1

        return candidates

    def _update_lock_state(self, candidates: list) -> Optional[dict]:
        if not candidates:
            if self.locked_track_id is not None:
                self.lock_miss_count += 1
                if self.lock_miss_count > self.max_lock_misses:
                    self.locked_track_id = None
                    self.lock_miss_count = 0
            return None

        candidate_by_id = {c["track_id"]: c for c in candidates}

        # Maintain lock while the same tracked object is still visible.
        if self.locked_track_id in candidate_by_id:
            self.lock_miss_count = 0
            return candidate_by_id[self.locked_track_id]

        # Locked object missing in current frame.
        if self.locked_track_id is not None:
            self.lock_miss_count += 1
            if self.lock_miss_count <= self.max_lock_misses:
                return None

        # Acquire new lock: largest area among stable IDs only.
        stable_candidates = [
            c
            for c in candidates
            if self.track_hit_counter.get(c["track_id"], 0) >= self.min_stable_hits
        ]
        if not stable_candidates:
            return None

        new_lock = max(stable_candidates, key=lambda x: x["area"])
        self.locked_track_id = new_lock["track_id"]
        self.lock_miss_count = 0
        return new_lock

    def _extract_track_id(self, det: Detection2D) -> Optional[str]:
        # `vision_msgs/Detection2D` may include `id` depending on message version/tooling.
        if hasattr(det, "id"):
            raw_id = str(det.id).strip()
            if raw_id:
                return raw_id
        return None

    def _extract_class_and_score(self, det: Detection2D) -> Tuple[Optional[int], float]:
        if not det.results:
            return None, 0.0

        best = max(det.results, key=lambda r: float(r.hypothesis.score))
        raw_class = str(best.hypothesis.class_id)
        try:
            class_id = int(raw_class)
        except ValueError:
            digits = "".join(c for c in raw_class if c.isdigit())
            if digits == "":
                return None, float(best.hypothesis.score)
            class_id = int(digits)

        return class_id, float(best.hypothesis.score)

    def _publish_target_vector(
        self, azimuth_norm: float, estimated_distance: float, lock_active: bool
    ) -> None:
        msg = Twist()
        msg.linear.x = float(azimuth_norm)      # normalized azimuth error [-1, 1]
        msg.linear.y = float(estimated_distance)  # distance-like heuristic (>0, closer => smaller)
        msg.linear.z = 1.0 if lock_active else 0.0  # lock status encoded as 1.0/0.0
        self.vector_pub.publish(msg)

    def _draw_locked_target(
        self,
        image: np.ndarray,
        candidate: dict,
        azimuth_norm: float,
        horizon_y: int,
    ) -> None:
        h, w = image.shape[:2]
        left, top, right, bottom = candidate["rect"]

        cv2.line(image, (0, horizon_y), (w, horizon_y), (0, 255, 255), 2)
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)

        cx = int(candidate["bbox"][0])
        cy = int(candidate["bbox"][1])
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(image, (w // 2, h), (w // 2, 0), (255, 255, 0), 1)

        text = (
            f"LOCK ID={candidate['track_id']} cls={candidate['class_id']} "
            f"az={azimuth_norm:+.3f}"
        )
        cv2.putText(
            image,
            text,
            (max(left, 5), max(top - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _is_detection_fresh(self, img_msg: Image, det_msg: Detection2DArray) -> bool:
        img_t = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        det_t = det_msg.header.stamp.sec + det_msg.header.stamp.nanosec * 1e-9

        if img_t <= 0.0 or det_t <= 0.0:
            return True

        return math.fabs(img_t - det_t) <= self.max_detection_age_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsvTargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
