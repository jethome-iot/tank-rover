#!/usr/bin/env bash
# Запускается ВНУТРИ контейнера (docker compose command). Вся ROS-часть ровера:
#   - лидар Oradar MS200 на /dev/ttyS3 (UART3, пины 16/18)
#   - VESC оба мотора на /dev/ttyS8 (UART8, пины 11/13), ТОК-режим, swap_sides
#   - twist mixer: /cmd_vel -> борта, max_duty=0.4 (потолок)
#   - ds4_receiver: UDP :9999 от геймпада на ПК -> /cmd_vel
#   - camera_bridge: RTP с хоста -> /camera/h264 (foxglove CompressedVideo)
#   - foxglove_bridge: всё наружу на :8765 для Lichtblick
#
# Камера ЭНКОДИТСЯ на ХОСТЕ (mpph264enc), сюда идёт H.264 по RTP.
# Хостовую часть (камера) поднимает tank_up.sh.
# Геймпад: sender на ПК (ds4_sender.py) шлёт UDP сюда, receiver кладёт в /cmd_vel.

set -o pipefail
source /opt/ros/jazzy/setup.bash
[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH=/root/ros_ws:$PYTHONPATH

# --- Лидар: UART3 (/dev/ttyS3, пины 16/18) ---
ros2 run oradar_lidar oradar_scan --ros-args \
    -p port_name:=/dev/ttyS3 \
    -p baudrate:=230400 \
    -p frame_id:=laser_frame \
    -p scan_topic:=scan \
    -p range_min:=0.05 \
    -p range_max:=12.0 \
    -p clockwise:=false \
    -p motor_speed:=10 &

# --- VESC: ток-режим, оба мотора, swap_sides. max_current под предел моторов (10А) ---
python3 /root/ros_ws/vesc_node.py --ros-args \
    -p port:=/dev/ttyS8 \
    -p can_id_b:=41 \
    -p swap_sides:=true \
    -p max_current:=8.0 \
    -p brake_current:=6.0 &

# --- Миксер: /cmd_vel -> борта. max_duty=0.4 потолок (геймпад дозирует внутри) ---
python3 /root/ros_ws/twist_mixer.py --ros-args \
    -p max_duty:=0.4 &

# --- Приёмник геймпада: UDP :9999 -> /cmd_vel ---
python3 /root/ros_ws/ds4_receiver.py --ros-args \
    -p port:=9999 &

# --- Камера: приём RTP с хоста -> foxglove CompressedVideo ---
python3 /root/ros_ws/camera_bridge.py --ros-args \
    -p udp_port:=5001 \
    -p frame_id:=camera &

# --- Мост наружу для Lichtblick ---
ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:=8765 \
    -p address:=0.0.0.0 \
    -p topic_whitelist:='["/scan","/camera/h264","/vesc/voltage","/vesc/rpm","/vesc/temp_mos","/tf","/tf_static","/rosout","/cmd_vel"]' &

wait
