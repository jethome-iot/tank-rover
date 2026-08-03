#!/usr/bin/env python3
"""Skid-steer миксер: /cmd_vel (Twist) -> /commands/left + /commands/right.

Танковое управление: линейная скорость едет прямо, угловая разводит борта.
  left  = linear - angular
  right = linear + angular
Оба борта нормируются, затем масштабируются max_duty.

Инверсия/своп бортов НЕ здесь, а в vesc_node (swap_sides/invert_*),
чтобы миксер оставался чистой кинематикой.

max_duty при ТОК-режиме vesc_node работает как множитель доли (0..1),
которая затем в ноде умножается на max_current. Т.е. потолок = max_duty * max_current.

Запуск:
  python3 twist_mixer.py --ros-args -p max_duty:=0.4
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TwistMixer(Node):
    def __init__(self):
        super().__init__('twist_mixer')
        self.declare_parameter('max_duty', 0.4)
        self.declare_parameter('turn_scale', 1.0)
        self.declare_parameter('invert_right', False)

        self.max_duty = float(self.get_parameter('max_duty').value)
        self.turn_scale = float(self.get_parameter('turn_scale').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        self.pub_l = self.create_publisher(Float64, 'commands/left', 10)
        self.pub_r = self.create_publisher(Float64, 'commands/right', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)

        self.get_logger().info(
            f'миксер: max_duty={self.max_duty} turn_scale={self.turn_scale} '
            f'invert_right={self.invert_right}')

    def on_twist(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z * self.turn_scale

        left = lin - ang
        right = lin + ang

        peak = max(abs(left), abs(right), 1.0)
        left /= peak
        right /= peak

        left *= self.max_duty
        right *= self.max_duty

        if self.invert_right:
            right = -right

        self.pub_l.publish(Float64(data=float(left)))
        self.pub_r.publish(Float64(data=float(right)))


def main():
    rclpy.init()
    node = TwistMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_l.publish(Float64(data=0.0))
            node.pub_r.publish(Float64(data=0.0))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
