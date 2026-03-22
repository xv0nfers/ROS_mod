from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="yolo_ros",
                executable="usv_opencv_tracker_node",
                name="usv_opencv_tracker_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw",
                        "target_vector_topic": "/usv/target_vector",
                        "debug_image_topic": "/usv/vision_debug",
                        "roi_top_ratio": 0.30,
                    }
                ],
            )
        ]
    )
