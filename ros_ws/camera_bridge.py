#!/usr/bin/env python3
"""Мост камера -> Foxglove CompressedVideo.

Энкод H.264 делает ХОСТ аппаратно (mpph264enc, /dev/mpp_service) и шлёт по
RTP/UDP на localhost. Эта нода (в контейнере) принимает RTP, собирает NAL-юниты
одного кадра и публикует foxglove_msgs/CompressedVideo -> Lichtblick.

Почему так: вендорский gstreamer1.0-rockchip (mpp) версии 1.14 несовместим по ABI
с GStreamer 1.24 в контейнере. Поэтому энкод остаётся на хосте, а в контейнере —
только приём и переупаковка (базовый GStreamer из Ubuntu, без mpp).

Хост шлёт так (аппаратный энкод):
  gst-launch-1.0 v4l2src device=/dev/video45 ! \
    image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! mppjpegdec ! \
    mpph264enc ! h264parse config-interval=1 ! \
    rtph264pay pt=96 config-interval=1 ! udpsink host=127.0.0.1 port=5001

Требования Foxglove CompressedVideo для h264:
  - Annex B (стартовые коды 00 00 00 01) -- rtph264depay так и отдаёт
  - каждое сообщение = ровно один кадр
  - keyframe (IDR) должен нести SPS/PPS -- config-interval=1 у пейлоадера
    вставляет SPS/PPS перед каждым IDR
  - без B-кадров -- mpph264enc их не генерит (baseline/main без B)

Запуск (в контейнере):
  python3 camera_bridge.py --ros-args -p udp_port:=5001 -p frame_id:=camera
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self.declare_parameter('udp_port', 5001)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('topic', 'camera/h264')

        self.frame_id = self.get_parameter('frame_id').value
        port = int(self.get_parameter('udp_port').value)
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(CompressedVideo, topic, 10)

        Gst.init(None)
        # udpsrc -> RTP H264 -> depay (отдаёт Annex-B access unit) -> appsink
        # каждый буфер из rtph264depay = один собранный access unit (кадр)
        pipeline_str = (
            f'udpsrc port={port} '
            f'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! '
            f'rtpjitterbuffer latency=50 ! '
            f'rtph264depay ! '
            f'video/x-h264,stream-format=byte-stream,alignment=au ! '
            f'appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true'
        )
        self.pipe = Gst.parse_launch(pipeline_str)
        self.sink = self.pipe.get_by_name('sink')
        self.sink.connect('new-sample', self.on_sample)
        self.pipe.set_state(Gst.State.PLAYING)

        self.get_logger().info(
            f'camera_bridge: RTP на :{port} -> {topic} (foxglove CompressedVideo h264)')
        self._n = 0
        self.create_timer(5.0, self._report)

    def on_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            data = bytes(mapinfo.data)   # Annex-B access unit (один кадр)
            msg = CompressedVideo()
            # timestamp из ROS-часов
            now = self.get_clock().now().to_msg()
            msg.timestamp = now
            msg.frame_id = self.frame_id
            msg.data = data
            msg.format = 'h264'
            self.pub.publish(msg)
            self._n += 1
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _report(self):
        self.get_logger().info(f'кадров опубликовано за 5с: {self._n}')
        self._n = 0

    def destroy_node(self):
        try:
            self.pipe.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
