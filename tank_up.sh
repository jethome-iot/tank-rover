#!/usr/bin/env bash
# Что поднимает:
#   1. Docker-контейнер (лидар + VESC + миксер + camera_bridge + foxglove)
#      -- через docker compose, ноды стартуют сами из tank_inside.sh
#   2. Аппаратный энкодер камеры на хосте (mpph264enc -> RTP на :5001)
#      -- его нельзя в контейнер (вендорский mpp 1.14 несовместим с gst 1.24)
#
# Управление с ноутбука:
#   Lichtblick -> ws://10.181.22.77 :8765  (видео + лидар + телеметрия)
#   teleop с клавы:  docker compose exec ros bash, затем
#                    ros2 run teleop_twist_keyboard teleop_twist_keyboard

set -e
cd "$(dirname "$0")"

IP=$(hostname -I | awk '{print $1}')

echo "[tank] waiting docker container up..."
docker compose up -d

echo "[tank] waiting camera_bridge (3с)..."
sleep 3

echo "[tank] waiting camera encoder..."
# gop=10 -> keyframe 3x/сек (меньше артефактов при потерях UDP)
# bps=4M -> щадящий битрейт для WiFi
gst-launch-1.0 v4l2src device=/dev/video45 ! \
  image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! mppjpegdec ! \
  mpph264enc gop=10 bps=4000000 ! h264parse config-interval=1 ! \
  rtph264pay pt=96 config-interval=1 ! udpsink host=127.0.0.1 port=5001 \
  > /tmp/tank_camera.log 2>&1 &
echo $! > /tmp/tank_camera.pid

echo ""
echo "[tank] READY."
echo ""
echo "  Launch gamepad with:"
echo "    python3 pc/ds4_sender.py --ip $IP"
echo ""
echo "  Lichtblick:       ws://$IP:8765"
