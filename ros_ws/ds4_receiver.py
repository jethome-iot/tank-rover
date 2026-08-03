#!/usr/bin/env python3
"""UDP-приёмник команд геймпада -> /cmd_vel. Запускается НА РОВЕРЕ (в контейнере).

Слушает UDP "linear,angular" от ds4_sender.py (на ПК), публикует Twist в /cmd_vel.
Дальше миксер -> VESC.

Failsafe: нет пакетов дольше timeout -> ноль (стоп). Дублирует deadman на ПК
и failsafe VESC, третий рубеж.

  python3 ds4_receiver.py --ros-args -p port:=9999
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Ds4Receiver(Node):
    def __init__(self):
        super().__init__('ds4_receiver')
        self.declare_parameter('port', 9999)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('max_lin', 1.0)
        self.declare_parameter('max_ang', 1.0)

        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        port = int(self.get_parameter('port').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.2)
        self.get_logger().info(f'ds4_receiver слушает UDP :{port}')

        self.lin = 0.0
        self.ang = 0.0
        self.last_rx = self.get_clock().now()
        self.lock = threading.Lock()

        threading.Thread(target=self._rx_loop, daemon=True).start()
        self.create_timer(0.05, self._publish)

    def _rx_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(64)
                parts = data.decode().strip().split(',')
                lin = float(parts[0])
                ang = float(parts[1])
                with self.lock:
                    self.lin = max(-1.0, min(1.0, lin))
                    self.ang = max(-1.0, min(1.0, ang))
                    self.last_rx = self.get_clock().now()
            except socket.timeout:
                continue
            except Exception:
                continue

    def _publish(self):
        with self.lock:
            dt = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
            lin, ang = self.lin, self.ang
        if dt > self.timeout:
            lin = ang = 0.0
        msg = Twist()
        msg.linear.x = lin * self.max_lin
        msg.angular.z = ang * self.max_ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Ds4Receiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
