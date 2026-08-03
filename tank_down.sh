#!/usr/bin/env bash

cd "$(dirname "$0")"

echo "[tank] гашу энкодер камеры..."
if [ -f /tmp/tank_camera.pid ]; then
    kill "$(cat /tmp/tank_camera.pid)" 2>/dev/null
    rm -f /tmp/tank_camera.pid
fi
# на всякий случай добить по имени
pkill -f 'gst-launch.*video45' 2>/dev/null

echo "[tank] останавливаю контейнер..."
docker compose down

echo "[tank] остановлено."
