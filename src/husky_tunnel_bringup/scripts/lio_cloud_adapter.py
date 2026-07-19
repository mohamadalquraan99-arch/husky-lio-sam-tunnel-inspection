#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class LioCloudAdapter(Node):

    def __init__(self):
        super().__init__("lio_cloud_adapter")

        self.publisher = self.create_publisher(
            PointCloud2,
            "/lio_sam/points",
            qos_profile_sensor_data,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/a200_0000/sensors/lidar3d_0/points",
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.reported_layout = False
        self.get_logger().info("LIO-SAM cloud adapter started")

    def cloud_callback(self, msg):
        if msg.point_step != 32:
            self.get_logger().error(
                f"Expected point_step 32, received {msg.point_step}"
            )
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        expected_size = msg.height * msg.row_step
        if raw.size < expected_size:
            self.get_logger().error("Point cloud data is smaller than expected")
            return

        rows = raw[:expected_size].reshape(msg.height, msg.row_step)
        records = rows[:, :msg.width * msg.point_step]
        records = records.reshape(-1, msg.point_step)

        float_type = np.dtype(">f4" if msg.is_bigendian else "<f4")
        xyz = records[:, :12].copy().view(float_type).reshape(-1, 3)

        valid_mask = np.isfinite(xyz).all(axis=1)
        filtered = records[valid_mask].copy()

        # Bytes 28–31 are padding in the original cloud.
        # Use them for LIO-SAM's per-point time field.
        filtered[:, 28:32] = 0

        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = int(filtered.shape[0])
        output.fields = list(msg.fields)
        output.fields.append(
            PointField(
                name="time",
                offset=28,
                datatype=PointField.FLOAT32,
                count=1,
            )
        )
        output.is_bigendian = msg.is_bigendian
        output.point_step = 32
        output.row_step = output.width * output.point_step
        output.data = filtered.tobytes()
        output.is_dense = True

        self.publisher.publish(output)

        if not self.reported_layout:
            removed = records.shape[0] - filtered.shape[0]
            self.get_logger().info(
                f"Input points: {records.shape[0]}, "
                f"valid points: {filtered.shape[0]}, "
                f"removed: {removed}"
            )
            self.reported_layout = True


def main(args=None):
    rclpy.init(args=args)
    node = LioCloudAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
