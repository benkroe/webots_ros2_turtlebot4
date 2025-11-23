#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from typing import Union, List
import random
import math
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity


def map_cliff_intensity(raw: Union[LaserScan, float],
                        min_range: float = 9.999999747378752e-05,
                        max_range: float = 0.15000000596046448,
                        invert: bool = True,
                        noise_std: float = 0.0) -> float:
    """
    Map a LaserScan (or raw distance) -> normalized intensity in [0,1].
    Uses same logic as irIntensityBridge: nearest valid range, normalize between
    min_range and max_range, invert so shorter distances => higher intensity.
    """
    if isinstance(raw, LaserScan):
        dists = [r for r in raw.ranges if r == r and r != float('inf') and r != -float('inf')]
        d = min(dists) if dists else float('inf')
    else:
        d = float(raw)

    if d != d or d == float('inf') or d == -float('inf') or max_range <= min_range:
        return 0.0

    norm = (d - min_range) / (max_range - min_range)
    norm = max(0.0, min(1.0, norm))
    intensity = 1.0 - norm if invert else norm

    if noise_std > 0.0:
        intensity += random.gauss(0.0, noise_std)
    return max(0.0, min(1.0, intensity))


class CliffIntensityBridge(Node):
    """
    Node runs in the 'Turtlebot4' namespace. Topics supplied below are relative,
    so they resolve to /Turtlebot4/...
    """
    def __init__(self,
                 sensor_topics: List[str] = None,
                 min_range: float = 9.999999747378752e-05,
                 max_range: float = 0.15000000596046448,
                 noise_std: float = 0.0):
        # set namespace to "Turtlebot4"
        super().__init__('cliff_intensity_bridge', namespace='Turtlebot4')

        if sensor_topics is None:
            sensor_topics = [
                'cliff_front_left',
                'cliff_front_right',
                'cliff_side_left',
                'cliff_side_right',
            ]
        self.sensor_topics = sensor_topics
        self.min_range = min_range
        self.max_range = max_range
        self.noise_std = noise_std

        # publisher is relative -> publishes to /Turtlebot4/cliff_intensity
        self.pub = self.create_publisher(IrIntensityVector, 'cliff_intensity', 10)
        self.latest = [0.0] * len(self.sensor_topics)

        for idx, topic in enumerate(self.sensor_topics):
            self.create_subscription(LaserScan, topic, self._make_cb(idx), 10)

        self.create_timer(0.1, self._publish)  # 10 Hz

    def _make_cb(self, idx):
        def cb(msg: LaserScan):
            self.latest[idx] = map_cliff_intensity(msg,
                                                   min_range=self.min_range,
                                                   max_range=self.max_range,
                                                   noise_std=self.noise_std)
        return cb

    def _publish(self):
        m = IrIntensityVector()
        # stamp header if present
        now = self.get_clock().now().to_msg()
        if hasattr(m, 'header'):
            try:
                m.header.stamp = now
            except Exception:
                pass

        # Build readings (IrIntensity[]) and assign to the known field 'readings'
        readings = []
        for v in self.latest:
            r = IrIntensity()
            if hasattr(r, 'header'):
                try:
                    r.header.stamp = now
                except Exception:
                    pass
            # scale normalized [0.0,1.0] to int16-like value (0..1000)
            ival = int(round(max(0.0, min(1.0, v)) * 1000.0))
            # clamp to int16 just in case
            if ival > 32767:
                ival = 32767
            if ival < -32768:
                ival = -32768
            r.value = ival
            readings.append(r)

        # assign to the proper field and publish
        try:
            m.readings = readings
        except Exception as e:
            self.get_logger().error(f'cliff_intensity_bridge: failed to assign readings: {e}')
            return

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = CliffIntensityBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()