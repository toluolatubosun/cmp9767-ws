#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math


class ClosestLaserPointPublisher(Node):
    def __init__(self):
        super().__init__('closest_laser_point_publisher')

        # Subscribe to laser scan data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Change if your laser topic name differs
            self.laser_callback,
            10
        )

        # Publisher for the closest point pose (for RViz)
        self.pose_publisher = self.create_publisher(PoseStamped, 'closest_point_pose', 10)

        # TF buffer & listener to transform poses between frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def laser_callback(self, scan_msg: LaserScan):
        """Process incoming laser scan and publish PoseStamped of closest detected point."""

        # Initialize variables for closest point search
        closest_distance = float('inf')
        closest_index = -1

        # Loop over all laser scan readings
        for index, distance in enumerate(scan_msg.ranges):
            if scan_msg.range_min < distance < scan_msg.range_max:
                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = index

        if closest_index == -1:
            self.get_logger().info("No valid laser scan readings detected.")
            return

        # Compute the angle of the closest beam
        beam_angle = scan_msg.angle_min + closest_index * scan_msg.angle_increment

        # Convert polar coordinates (r, θ) to Cartesian (x, y) in the laser frame
        x_in_laser_frame = closest_distance * math.cos(beam_angle)
        y_in_laser_frame = closest_distance * math.sin(beam_angle)

        # Build a PoseStamped message for the closest point
        closest_point_pose = PoseStamped()
        closest_point_pose.header = scan_msg.header
        closest_point_pose.header.frame_id = scan_msg.header.frame_id  # usually "laser_link"
        closest_point_pose.pose.position.x = x_in_laser_frame
        closest_point_pose.pose.position.y = y_in_laser_frame
        closest_point_pose.pose.position.z = 0.0  # Laser is 2D, so z = 0
        closest_point_pose.pose.orientation.w = 1.0  # Neutral orientation

        # Try to transform this pose into the 'odom' frame for visualization in RViz
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',  # Target frame (global)
                closest_point_pose.header.frame_id,  # Source frame (laser)
                rclpy.time.Time()
            )

            # Transform the pose to the odom frame
            transformed_pose = do_transform_pose(closest_point_pose, transform)
            self.pose_publisher.publish(transformed_pose)

            self.get_logger().info(
                f"Closest object detected at {closest_distance:.2f} m "
                f"→ Pose in 'odom': x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}"
            )

        except Exception as error:
            # If TF lookup fails, publish in the original laser frame
            self.pose_publisher.publish(closest_point_pose)
            self.get_logger().warn(f"Could not transform pose to 'odom': {error}")
            self.get_logger().info(
                f"Published closest point in laser frame '{closest_point_pose.header.frame_id}' "
                f"at x={x_in_laser_frame:.2f}, y={y_in_laser_frame:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ClosestLaserPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# How to visualize:
# - In RViz, add a Pose display.
# - Set the topic to /closest_point_pose.
# - You should see a marker showing the closest detected obstacle, updated live.