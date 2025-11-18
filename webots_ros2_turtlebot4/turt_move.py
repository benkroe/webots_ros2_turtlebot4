# ...existing code...
#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopASWD(Node):
    """
    Simple terminal teleop: w/s = forward/back, a/d = turn left/right,
    space = stop, q = quit. Publishes geometry_msgs/Twist to /cmd_vel.
    Run this in a dedicated terminal.
    """
    def __init__(self):
        super().__init__('teleop_aswd')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # configurable speeds
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('pub_rate', 10.0)  # Hz
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.pub_rate = float(self.get_parameter('pub_rate').value)
        self.current_twist = Twist()
        self.done = False

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            print("Teleop ASWD started. Keys: w/s forward/back, a/d rotate, space stop, q quit")
            rate = self.create_rate(self.pub_rate)
            while rclpy.ok() and not self.done:
                # non-blocking read
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    c = sys.stdin.read(1)
                    self._handle_key(c)
                # publish current twist
                self.pub.publish(self.current_twist)
                rclpy.spin_once(self, timeout_sec=0.0)
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # ensure robot stops
            stop = Twist()
            self.pub.publish(stop)

    def _handle_key(self, c: str):
        c = c.lower()
        t = Twist()
        if c == 'w':
            t.linear.x = self.linear_speed
        elif c == 's':
            t.linear.x = -self.linear_speed
        elif c == 'a':
            t.angular.z = self.angular_speed
        elif c == 'd':
            t.angular.z = -self.angular_speed
        elif c == ' ':
            t = Twist()
        elif c == 'q':
            self.done = True
            return
        else:
            # ignore other keys
            return
        self.current_twist = t

def main(args=None):
    rclpy.init(args=args)
    node = TeleopASWD()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
