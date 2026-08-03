#!/usr/bin/env python3
"""VESC driver на pyserial для ROS 2 — двухмоторный (dual FSESC), CAN forwarding.
Режим управления: ТОК (SET_CURRENT) для тяги + SET_CURRENT_BRAKE для стопа/failsafe.

Почему current, а не duty:
  - тяга задаётся ТОКОМ (момент ~ ток), мягкий старт, честное усилие
  - на препятствии момент держится, мотор продавливает вместо срыва оборотов
Важно про поведение:
  - удержание стика = постоянный ток = постоянный МОМЕНТ (разгон до баланса с
    трением), это "педаль ускорения", не "педаль скорости"
  - стоп/центр/failsafe шлёт SET_CURRENT_BRAKE (активный тормоз), а НЕ ток=0,
    иначе был бы накат (freewheel)

БЕЗОПАСНОСТЬ ТОКА:
  max_current по умолчанию задан КОНСЕРВАТИВНО. Подстрой под предел моторов!
  Слишком большой ток = перегрев/повреждение. Начинай с малого, повышай осторожно.

Топология Flipsky Dual FSESC 4.20:
  половина A — UART (эта плата), половина B — CAN forwarding (can_id_b)
  swap_sides / invert_* — под геометрию шасси (подобрано: swap=true)

Смещения телеметрии выверены по FW 6.6: v_in=24.8В, temp=32.5C.

Топики:
  вход:  /commands/left, /commands/right   Float64  -1..1 (доля от max_current)
  выход: /vesc/voltage /vesc/temp_mos /vesc/rpm /vesc/current
"""

import struct
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32, Float64

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_FORWARD_CAN = 34


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(payload: bytes) -> bytes:
    c = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([c >> 8, c & 0xFF, 0x03])


def unpack(buf: bytes):
    if len(buf) < 5 or buf[0] != 0x02:
        return None
    length = buf[1]
    if len(buf) < length + 5:
        return None
    payload = buf[2:2 + length]
    crc_rx = (buf[2 + length] << 8) | buf[3 + length]
    if buf[4 + length] != 0x03 or crc16(payload) != crc_rx:
        return None
    return payload


class VescNode(Node):
    def __init__(self):
        super().__init__('vesc')
        self.declare_parameter('port', '/dev/ttyS8')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('can_id_b', 41)
        self.declare_parameter('swap_sides', True)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        # ТОК: подстрой под свои моторы! Консервативный дефолт.
        self.declare_parameter('max_current', 8.0)     # А, тяга на полном стике
        self.declare_parameter('brake_current', 6.0)   # А, сила торможения на стопе
        self.declare_parameter('poll_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        self.can_b = int(self.get_parameter('can_id_b').value)
        self.swap = bool(self.get_parameter('swap_sides').value)
        self.inv_l = bool(self.get_parameter('invert_left').value)
        self.inv_r = bool(self.get_parameter('invert_right').value)
        self.max_current = float(self.get_parameter('max_current').value)
        self.brake_current = float(self.get_parameter('brake_current').value)

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(
            f'VESC {port} | ТОК режим | max_current={self.max_current}A '
            f'brake={self.brake_current}A swap={self.swap} can_b={self.can_b}')

        self.pub_v = self.create_publisher(Float32, 'vesc/voltage', 10)
        self.pub_t = self.create_publisher(Float32, 'vesc/temp_mos', 10)
        self.pub_rpm = self.create_publisher(Float32, 'vesc/rpm', 10)
        self.pub_cur = self.create_publisher(Float32, 'vesc/current', 10)

        self.create_subscription(Float64, 'commands/left', self.on_left, 10)
        self.create_subscription(Float64, 'commands/right', self.on_right, 10)

        self.last_cmd_t = self.get_clock().now()
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.lock = threading.Lock()

        # последние команды бортов, чтобы failsafe знал что тормозить
        self._left = 0.0
        self._right = 0.0

        period = 1.0 / float(self.get_parameter('poll_hz').value)
        self.create_timer(period, self.poll)
        self.create_timer(0.1, self.failsafe_check)
        self._ok = 0
        self._bad = 0
        self.create_timer(5.0, self.report)

    def _send(self, payload: bytes):
        with self.lock:
            self.ser.write(pack(payload))

    # --- низкоуровневые команды ---
    def _current_payload(self, amps: float) -> bytes:
        return bytes([COMM_SET_CURRENT]) + struct.pack('>i', int(amps * 1000))

    def _brake_payload(self, amps: float) -> bytes:
        return bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack('>i', int(amps * 1000))

    def _drive_uart(self, payload: bytes):
        self._send(payload)

    def _drive_can(self, payload: bytes):
        self._send(bytes([COMM_FORWARD_CAN, self.can_b]) + payload)

    # --- логика борта: тяга током или тормоз ---
    def _apply_side(self, frac: float, is_uart: bool):
        """frac -1..1. Ненулевой -> ток (тяга). Ноль -> тормоз."""
        if abs(frac) < 0.02:
            payload = self._brake_payload(self.brake_current)
        else:
            frac = max(-1.0, min(1.0, frac))
            payload = self._current_payload(frac * self.max_current)
        if is_uart:
            self._drive_uart(payload)
        else:
            self._drive_can(payload)

    def set_left(self, frac: float):
        self._left = frac
        if self.inv_l:
            frac = -frac
        # swap решает, левый борт на UART или на CAN
        self._apply_side(frac, is_uart=not self.swap)

    def set_right(self, frac: float):
        self._right = frac
        if self.inv_r:
            frac = -frac
        self._apply_side(frac, is_uart=self.swap)

    def on_left(self, msg):
        self.last_cmd_t = self.get_clock().now()
        self.set_left(msg.data)

    def on_right(self, msg):
        self.last_cmd_t = self.get_clock().now()
        self.set_right(msg.data)

    def failsafe_check(self):
        dt = (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            # нет команд -> АКТИВНЫЙ ТОРМОЗ обоих бортов (не накат!)
            self.set_left(0.0)
            self.set_right(0.0)

    def poll(self):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(pack(bytes([COMM_GET_VALUES])))
            buf = self.ser.read(128)
        p = unpack(buf)
        if p is None or len(p) < 30 or p[0] != COMM_GET_VALUES:
            self._bad += 1
            return
        self._ok += 1
        temp_mos = struct.unpack_from('>h', p, 1)[0] / 10.0
        cur_motor = struct.unpack_from('>i', p, 5)[0] / 100.0
        rpm = struct.unpack_from('>i', p, 23)[0]
        v_in = struct.unpack_from('>h', p, 27)[0] / 10.0
        self.pub_v.publish(Float32(data=float(v_in)))
        self.pub_t.publish(Float32(data=float(temp_mos)))
        self.pub_rpm.publish(Float32(data=float(rpm)))
        self.pub_cur.publish(Float32(data=float(cur_motor)))

    def report(self):
        total = self._ok + self._bad
        if total:
            self.get_logger().info(f'телеметрия ok={self._ok} bad={self._bad}')
        self._ok = self._bad = 0


def main():
    rclpy.init()
    node = VescNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # на выходе — тормоз, потом отпустить
            node.set_left(0.0)
            node.set_right(0.0)
        except Exception:
            pass
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
