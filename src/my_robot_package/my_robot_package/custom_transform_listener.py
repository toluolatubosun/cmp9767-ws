#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class CustomTransformListener(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def timer_callback(self):
        try:
            # Get transform from odom → camera frame
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'depth_camera_link',
                rclpy.time.Time()
            )

            # Extract the necessary data from the transform object
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            child_frame_id = transform.child_frame_id
            header = transform.header

            # Print the received transformation information
            self.get_logger().info(
                f"Received TF Transform:\n"
                f"  Child frame ID: {child_frame_id}\n"
                f"  Translation: [x: {translation.x}, y: {translation.y}, z: {translation.z}]\n"
                f"  Rotation: [x: {rotation.x}, y: {rotation.y}, z: {rotation.z}, w: {rotation.w}]\n"
                f"  Frame ID: {header.frame_id}\n"
                f"  Timestamp: {header.stamp.sec}.{header.stamp.nanosec}")

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {str(e)}")

def main():
    rclpy.init()
    node = CustomTransformListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
