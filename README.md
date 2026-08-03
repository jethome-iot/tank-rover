# Tank Rover — NanoPi M5 (RK3576)

Телеуправляемый гусеничный (skid-steer) ровер на базе NanoPi M5 (Rockchip RK3576).
ROS 2 Jazzy в Docker, аппаратное H.264-видео, лидар, два мотора через VESC,
управление с геймпада DualShock 4. Картинка + лидар + телеметрия в Lichtblick.

## Возможности

- **Камера** — аппаратный H.264 (VPU RK3576, `mpph264enc`), CPU почти не грузится
- **Лидар** — Oradar MS200, публикует `/scan`
- **Приводы** — два мотора на сдвоенном Flipsky FSESC 4.20, свой pyserial-драйвер
  VESC (ток-режим), второй мотор через CAN forwarding
- **Управление** — DualShock 4 (гоночная схема: R2 газ, L2 назад, стик руль),
  регулировка мощности крестовиной; плюс телеоп с клавиатуры
- **Визуализация** — Foxglove/Lichtblick: видео (CompressedVideo) + лидар + телеметрия
- Всё поднимается одной командой, автозапуск в контейнере

## Архитектура

```
                    ХОСТ (NanoPi M5, Debian, BSP-ядро 6.1)
  ┌───────────────────────────────────────────────────────────┐
  │  камера USB → GStreamer (mppjpegdec ! mpph264enc)          │
  │            → RTP → udp://127.0.0.1:5001                     │
  │                                                            │
  │  Docker-контейнер (ROS 2 Jazzy):                           │
  │    lidar (ttyS3) ── /scan                                  │
  │    vesc_node (ttyS8) ── /vesc/* , слушает /commands/{left,right}
  │    twist_mixer ── /cmd_vel → /commands/{left,right}        │
  │    ds4_receiver (udp:9999) ── /cmd_vel                     │
  │    camera_bridge (udp:5001) ── /camera/h264 (CompressedVideo)
  │    foxglove_bridge ── ws://0.0.0.0:8765                    │
  └───────────────────────────────────────────────────────────┘
                    │ WiFi                     │ WiFi
              ws://…:8765                 udp://…:9999
                    │                          │
              [ПК] Lichtblick          [ПК] ds4_sender.py (геймпад)
```

Камера кодируется на **хосте** (вендорский `gstreamer1.0-rockchip` mpp 1.14
несовместим по ABI с GStreamer 1.24 в контейнере), в контейнер приходит уже
готовый H.264 по RTP — `camera_bridge` переупаковывает его в `CompressedVideo`.

## Карта портов (RK3576, важно!)

Номер `/dev/ttySN` не совпадает с номером пина. Установлено эмпирически + по DTS:

| Устройство | Порт        | UART  | Пины разъёма | Скорость |
|------------|-------------|-------|--------------|----------|
| VESC       | `/dev/ttyS8`| UART8 | 11 / 13      | 115200   |
| Лидар      | `/dev/ttyS3`| UART3 | 16 / 18      | 230400   |

Оба UART уже разведены FriendlyELEC (`rk3576-nanopi5-rev01.dts`), device tree
править не требовалось. Пин 16 = UART3_TX (→ RX лидара), пин 18 = UART3_RX (→ TX лидара).

## Требования

**Ровер (NanoPi M5):**
- Debian (FriendlyELEC), вендорское BSP-ядро (нужно для `mpph264enc`)
- Docker + docker compose
- GStreamer с плагином `gstreamer1.0-rockchip` (mpp) на хосте — для аппаратного H.264
- рабочий Python-модуль `pyserial` (для локальных тестов VESC)

**ПК:**
- Lichtblick (или Foxglove Studio)
- Python + `pygame` (для геймпада): `sudo apt install python3-pygame`

## Установка

1. Склонировать репозиторий на ровер в `~/tank`:
   ```bash
   git clone <repo> ~/tank && cd ~/tank
   ```

2. **Драйвер лидара** (не в репозитории — вендорский). Скачать `MS200_Ros-V1.3.4`,
   распаковать в `ros_ws/src/`, собрать под colcon:
   ```
   set(COMPILE_METHOD COLCON)   # в CMakeLists.txt
   cp package_ros2.xml package.xml
   ```
   (пакет `oradar_lidar`, executable `oradar_scan`)

3. **pyserial в контейнере** — в образ уже добавлен `python3-serial` (см. Dockerfile).
   Если запускаешь ноды вне готового образа, положи модуль `serial` в `ros_ws/`
   и добавь `PYTHONPATH=/root/ros_ws`.

4. Собрать образ (легаси-билдер, buildx на устройстве старый):
   ```bash
   DOCKER_BUILDKIT=0 docker build --network=host -t tank-ros:local .
   ```

## Запуск

Одной командой на ровере:
```bash
./tank_up.sh
```
Поднимает контейнер (все ROS-ноды) + хостовый энкодер камеры. Выведет адрес Lichtblick.

Остановить:
```bash
./tank_down.sh
```

**Lichtblick** (на ПК): подключиться к `ws://<ip-ровера>:8765`, добавить панели
Image (`/camera/h264`) и 3D (`/scan`).

**Геймпад** (на ПК): `python3 pc/ds4_sender.py --ip <ip-ровера>`

**Клавиатура** (альтернатива геймпаду):
```bash
docker compose exec ros bash -ic "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

## Управление с геймпада (DualShock 4)

| Орган            | Действие                    |
|------------------|-----------------------------|
| R2               | газ вперёд (аналоговый)      |
| L2               | задний ход (аналоговый)     |
| левый стик X     | поворот                     |
| крестовина ↑/↓   | потолок мощности ±5%         |

Отпустил триггеры — стоп (failsafe на трёх уровнях: пустая команда в sender,
таймаут в ds4_receiver 0.5с, таймаут в vesc_node 0.5с → тормоз).

## Управление приводами (VESC)

Ток-режим: тяга через `SET_CURRENT`, стоп/failsafe через `SET_CURRENT_BRAKE`
(активное торможение, не накат). Ключевые параметры `vesc_node`:

- `max_current` — потолок тока, А (ПОД ПРЕДЕЛ МОТОРА! у нас моторы 10А → ставим 8)
- `brake_current` — сила торможения, А
- `swap_sides` — какой мотор какой борт (подобрано `true`)
- `can_id_b` — CAN ID второй половины (у нас 41)

Итоговый ток = `стик × уровень(крестовина) × max_duty(миксер) × max_current`.

Есть также duty-версия драйвера в истории проекта (проще, тормозит на отпускании,
предсказуемая скорость) — переключается сменой команд на `SET_DUTY`.

## Файлы

```
Dockerfile              образ ROS 2 Jazzy + все зависимости
docker-compose.yml      сервис ros, проброс ttyS3/ttyS8, host-сеть
tank_up.sh              запуск всего (хост): контейнер + энкодер камеры
tank_down.sh            остановка
ros_ws/
  tank_inside.sh        автозапуск ROS-нод внутри контейнера
  vesc_node.py          драйвер VESC (ток-режим, dual через CAN)
  twist_mixer.py        skid-steer: /cmd_vel → борта
  ds4_receiver.py       приём UDP от геймпада → /cmd_vel
  camera_bridge.py      RTP H.264 → foxglove CompressedVideo
pc/
  ds4_sender.py         читалка DualShock4 на ПК → UDP на ровер
```

## Заметки

- **Почему свой VESC-драйвер:** f1tenth/vesc использует транспорт `serial_driver`
  (Autoware, ASIO), который на нативном UART RK3576 (dw-apb-uart, DMA недоступен →
  interrupt mode) молча не читает порт. pyserial с блокирующим чтением работает.
- **Телеметрия VESC** (offsets FW 6.6): temp_mos `[1:3]/10`, ток `[5:9]/100`,
  rpm `[23:27]`, v_in `[27:29]/10`.
- **CAN forwarding:** команда второй половине оборачивается в `COMM_FORWARD_CAN <id>`.
- **Camera bridge:** Foxglove требует Annex-B, один кадр на сообщение, SPS перед
  каждым IDR (`config-interval=1`), без B-кадров (mpph264enc их не даёт). Против
  артефактов на движении: `mpph264enc gop=10 bps=4000000`.
