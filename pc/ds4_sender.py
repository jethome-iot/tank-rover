#!/usr/bin/env python3
"""Читалка DualShock4 для ПК -> UDP на ровер. ГОНОЧНАЯ схема. НА ПК (ROS не нужен).

  sudo apt install python3-pygame
  python3 ds4_sender.py --ip <IP-ровера>

Управление (как в гонках, руки на "руле"):
  R2 (ось 5)             газ ВПЕРЁД, аналоговый (чуть-полный)
  L2 (ось 2)             задний ход, аналоговый
  левый стик X (ось 0)   поворот (руль)
  крестовина вверх/вниз  потолок мощности +/- 5%
  БЕЗ deadman: отпустил триггеры -> стоп (сами как педали)

Триггеры DS4: в покое ось = -1, вжат = +1. Пересчёт (ось+1)/2 -> 0..1.
Итог: linear = (R2 - L2), кладётся в /cmd_vel; поворот = стик X.
На ровере: /cmd_vel -> миксер (max_duty потолок) -> VESC (ток).
"""

import argparse
import socket
import sys
import time

import pygame

AX_STEER = 0         # левый стик горизонталь -> поворот
AX_L2 = 2            # L2 -> назад
AX_R2 = 5            # R2 -> вперёд
DEADZONE = 0.08

LEVEL_START = 0.20
LEVEL_STEP = 0.05
LEVEL_MIN = 0.05
LEVEL_MAX = 1.00


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', required=True, help='IP ровера')
    ap.add_argument('--port', type=int, default=9999)
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Геймпад не найден. Воткнут ли DS4?")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Геймпад: {js.get_name()}")
    print(f"Шлю на {args.ip}:{args.port}. R2=газ, L2=назад, левый стик=руль.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / args.rate

    def dz(v):
        return 0.0 if abs(v) < DEADZONE else v

    def trig(ax):
        # триггер: покой -1 -> 0, вжат +1 -> 1
        return (js.get_axis(ax) + 1.0) / 2.0

    level = LEVEL_START
    print(f"уровень мощности: {int(level*100)}%")
    hat_prev = 0

    try:
        while True:
            pygame.event.pump()

            # --- крестовина: потолок мощности ---
            _, hat_y = js.get_hat(0)
            if hat_y == 1 and hat_prev != 1:
                level = min(LEVEL_MAX, level + LEVEL_STEP)
                print(f"уровень мощности: {int(round(level*100))}%")
            elif hat_y == -1 and hat_prev != -1:
                level = max(LEVEL_MIN, level - LEVEL_STEP)
                print(f"уровень мощности: {int(round(level*100))}%")
            hat_prev = hat_y

            # --- газ/тормоз с триггеров ---
            fwd = trig(AX_R2)      # 0..1 вперёд
            rev = trig(AX_L2)      # 0..1 назад
            lin = (fwd - rev) * level     # вперёд минус назад

            # --- руль с левого стика ---
            ang = -dz(js.get_axis(AX_STEER)) * level   # знак под skid-steer

            sock.sendto(f"{lin:.3f},{ang:.3f}".encode(), (args.ip, args.port))
            time.sleep(dt)
    except KeyboardInterrupt:
        sock.sendto(b"0,0", (args.ip, args.port))
        print("\nстоп")


if __name__ == '__main__':
    main()
