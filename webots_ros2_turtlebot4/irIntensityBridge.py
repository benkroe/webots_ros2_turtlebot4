#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
from typing import Union
from sensor_msgs_py import point_cloud2 as pc2
import random
import math

# ...existing code...

def map_ir_intensity(raw: Union[PointCloud2, float],
                     min_range: float = 0.02,
                     max_range: float = 0.20,
                     invert: bool = True,
                     noise_std: float = 0.0) -> float:
    if isinstance(raw, PointCloud2):
        distances = []
        for p in pc2.read_points(raw, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if x is None or y is None or z is None:
                continue
            distances.append(math.sqrt(x * x + y * y + z * z))
        d = min(distances) if distances else float('inf')
    else:
        d = float(raw)

    if d != d or d == float('inf') or d == -float('inf'):
        return 0.0

    if max_range <= min_range:
        intensity = 0.0
    else:
        normalized = (d - min_range) / (max_range - min_range)
        normalized = max(0.0, min(1.0, normalized))
        intensity = 1.0 - normalized if invert else normalized

    if noise_std and noise_std > 0.0:
        intensity += random.gauss(0.0, noise_std)

    return max(0.0, min(1.0, intensity))


class IrIntensityBridge(Node):
    """
    Node runs in the 'Turtlebot4' namespace. Topics supplied below are relative,
    so they resolve to /Turtlebot4/...
    Publishes irobot_create_msgs/IrIntensityVector on 'ir_intensity'.
    Each reading.value is in [0.0, 1.0].
    """
    def __init__(self,
                 sensor_topics=None,
                 min_range=0.02,
                 max_range=0.20,
                 noise_std=0.0):

        super().__init__('ir_intensity_bridge')

        if sensor_topics is None:
            # relative topic names -> will resolve to /Turtlebot4/...
            sensor_topics = [
                'ir_intensity_front_center_left/point_cloud',
                'ir_intensity_front_center_right/point_cloud',
                'ir_intensity_front_left/point_cloud',
                'ir_intensity_front_right/point_cloud',
                'ir_intensity_left/point_cloud',
                'ir_intensity_right/point_cloud',
                'ir_intensity_side_left/point_cloud',
            ]
        self.sensor_topics = sensor_topics
        self.min_range = min_range
        self.max_range = max_range
        self.noise_std = noise_std

        self.pub = self.create_publisher(IrIntensityVector, 'ir_intensity', 10)
        self.latest = [0.0] * len(self.sensor_topics)

        for idx, topic in enumerate(self.sensor_topics):
            self.create_subscription(PointCloud2, topic, self._make_cb(idx), 10)

        self.create_timer(0.1, self._publish)  # 10 Hz

    def _make_cb(self, idx):
        def cb(msg: PointCloud2):
            val = map_ir_intensity(msg, min_range=self.min_range,
                                   max_range=self.max_range,
                                   noise_std=self.noise_std)
            self.latest[idx] = val
        return cb

    def _publish(self):
        m = IrIntensityVector()
        m.readings = []
        for v in self.latest:
            r = IrIntensity()
            try:
                # clamp normalized float to [0.0, 1.0]
                fv = max(0.0, min(1.0, float(v)))
            except Exception:
                fv = 0.0
            if hasattr(r, 'value'):
                try:
                    r.value = int(round(fv * 255.0))
                except Exception:
                    r.value = 0
            if hasattr(r, 'intensity'):
                try:
                    r.intensity = float(fv)
                except Exception:
                    r.intensity = 0.0
            m.readings.append(r)
        self.pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = IrIntensityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ir_intensity_bridge node.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
