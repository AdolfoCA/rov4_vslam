#!/usr/bin/env python3
"""ROS 2 driver for the BlueROV2's monocular camera, received as H.264 over UDP.

How the video actually reaches topside
--------------------------------------
The BlueROV2's low-light USB camera is plugged into the ROV's Raspberry Pi. BlueOS
encodes it to H.264 on the Pi (the camera itself does the encoding on newer units) and
sends it as an RTP stream over UDP to the topside computer - by default to port 5600.
Nothing on this end needs to request the stream; the packets simply arrive.

So the driver is a GStreamer pipeline whose sink is an ``appsink`` we pull frames from:

    udpsrc -> rtpjitterbuffer -> rtph264depay -> h264parse -> avdec_h264
           -> videoconvert -> BGR appsink

``rtpjitterbuffer`` is what makes the difference between usable and unwatchable video
on a tether: UDP packets arrive out of order and the decoder needs them in sequence.
Its ``latency`` is a deliberate trade - more buffering absorbs more jitter but delays
every frame - so it is exposed as a parameter.

``sync=false`` plus ``drop=true`` with a shallow queue on the appsink means the node
always works on the newest frame and throws away anything it could not keep up with.
For a pilot's view or a SLAM front end, a fresh frame matters more than a complete
sequence.

Timestamps
----------
Frames are stamped on arrival in the ROS clock. The stream carries RTP timestamps, but
they are relative to an arbitrary sender epoch with no RTCP sender reports to anchor
them, so they cannot be converted to a shared time base. The encode + tether + decode
latency (tens of milliseconds, and variable) is therefore folded into the stamp. If you
need the camera tightly synchronised with the IMU for visual-inertial work, budget for
an explicit offset calibration; the ``timestamp_offset_s`` parameter exists so you can
apply the result without touching this code.
"""

from __future__ import annotations

import array
import threading
from typing import Optional

# GStreamer MUST be imported before rclpy. Importing gi/Gst *after* rclpy leaves the
# process in a state where the very next rclpy Node() construction segfaults inside the
# _rclpy C extension (rclpy/node.py:175), with no Python-level error. Verified on
# ros:humble-ros-base: gi-then-rclpy works, rclpy-then-gi crashes. Keep this block first.
try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import GLib, Gst, GstApp  # noqa: F401 - GstApp registers appsink
except (ImportError, ValueError) as exc:  # pragma: no cover
    raise ImportError(
        "PyGObject with GStreamer 1.0 typelibs is required. Inside the container these "
        "come from python3-gi and gir1.2-gst-plugins-base-1.0."
    ) from exc


import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from bluerov2_camera.calibration import load_camera_info, uncalibrated_camera_info

try:
    import cv2
except ImportError:  # pragma: no cover - only needed for JPEG republishing
    cv2 = None


# {codec} is "h264" (BlueOS / BlueROV2 camera) or "h265" (the Blue Atlas multi-camera
# system); the depayloader, parser and decoder are all named after it.
DEFAULT_PIPELINE = (
    "udpsrc port={port} "
    'caps="application/x-rtp,media=video,encoding-name={encoding},payload=96,clock-rate=90000" '
    "! rtpjitterbuffer latency={latency_ms} drop-on-latency=true "
    "! rtp{codec}depay "
    "! {codec}parse "
    "! avdec_{codec} output-corrupt=false "
    "! videoconvert "
    "! video/x-raw,format=BGR "
    "! appsink name=ros_sink emit-signals=true sync=false max-buffers=2 drop=true"
)


class BlueRov2CameraNode(Node):
    """Decode the BlueROV2 UDP H.264 stream and publish it as ROS images."""

    def __init__(self) -> None:
        super().__init__("camera_node")

        # --- Parameters ------------------------------------------------------------
        # TODO(deployment): 5600 is the BlueOS default for the first video stream. If
        # you have more than one camera, or you changed the stream configuration in the
        # BlueOS video page, set the matching port here.
        self.declare_parameter("udp_port", 5600)
        self.declare_parameter("jitter_buffer_ms", 50)
        # "h264" for the BlueROV2 camera via BlueOS, "h265" for the multi-camera system.
        self.declare_parameter("codec", "h264")

        # Set this to override the whole pipeline, for example to pull RTSP instead:
        #   rtspsrc location=rtsp://192.168.2.2:8554/video latency=100 ! rtph264depay
        #   ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR
        #   ! appsink name=ros_sink emit-signals=true sync=false max-buffers=2 drop=true
        # The pipeline must contain an appsink named 'ros_sink' producing BGR frames.
        self.declare_parameter("pipeline_override", "")

        self.declare_parameter("frame_id", "camera_link_optical")
        self.declare_parameter("camera_name", "bluerov2_lowlight")

        # TODO(deployment): point this at a real calibration produced by
        # camera_calibration for THIS camera in THIS housing. A flat port in water
        # changes the effective focal length by roughly the refractive index of water
        # (about 1.33), so an in-air calibration is not transferable - calibrate wet.
        self.declare_parameter("camera_info_url", "")

        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("timestamp_offset_s", 0.0)

        self.declare_parameter("watchdog_period_s", 5.0)
        self.declare_parameter("restart_after_silence_s", 10.0)

        p = self.get_parameter
        self._frame_id: str = p("frame_id").value
        self._timestamp_offset_ns = int(float(p("timestamp_offset_s").value) * 1e9)
        self._publish_compressed = bool(p("publish_compressed").value)
        self._jpeg_quality = int(p("jpeg_quality").value)

        if self._publish_compressed and cv2 is None:
            self.get_logger().warning(
                "publish_compressed is set but OpenCV is not importable; disabling it"
            )
            self._publish_compressed = False

        # --- Publishers --------------------------------------------------------------
        self._pub_image = self.create_publisher(
            Image, "camera/image_raw", qos_profile_sensor_data
        )
        self._pub_info = self.create_publisher(
            CameraInfo, "camera/camera_info", qos_profile_sensor_data
        )
        self._pub_compressed = self.create_publisher(
            CompressedImage, "camera/image_raw/compressed", qos_profile_sensor_data
        )

        # --- Calibration ---------------------------------------------------------------
        self._camera_info: Optional[CameraInfo] = None
        info_url = str(p("camera_info_url").value)
        if info_url:
            path = info_url[len("file://"):] if info_url.startswith("file://") else info_url
            self._camera_info = load_camera_info(path, self._frame_id)
            if self._camera_info is None:
                self.get_logger().warning(
                    f"Could not read a camera calibration from '{path}'; publishing "
                    f"CameraInfo with zero intrinsics until one is provided"
                )
            else:
                self.get_logger().info(f"Loaded camera calibration from '{path}'")
        else:
            self.get_logger().warning(
                "No camera_info_url set - CameraInfo will carry the image size only. "
                "Anything that needs metric vision will need a real calibration."
            )

        # --- State --------------------------------------------------------------------
        self._frame_count = 0
        self._frames_since_report = 0
        self._counter_lock = threading.Lock()
        self._last_frame_time = self.get_clock().now()

        Gst.init(None)
        self._pipeline: Optional[Gst.Pipeline] = None
        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()

        self._start_pipeline()
        self._watchdog = self.create_timer(
            float(p("watchdog_period_s").value), self._check_stream
        )

    # -- Pipeline management --------------------------------------------------------

    def _pipeline_description(self) -> str:
        override = str(self.get_parameter("pipeline_override").value).strip()
        if override:
            return override
        codec = str(self.get_parameter("codec").value).strip().lower()
        if codec not in ("h264", "h265"):
            raise RuntimeError(f"Unsupported codec '{codec}': use 'h264' or 'h265'")
        return DEFAULT_PIPELINE.format(
            port=int(self.get_parameter("udp_port").value),
            latency_ms=int(self.get_parameter("jitter_buffer_ms").value),
            codec=codec,
            encoding=codec.upper(),
        )

    def _start_pipeline(self) -> None:
        description = self._pipeline_description()
        self.get_logger().info(f"Starting GStreamer pipeline: {description}")

        try:
            pipeline = Gst.parse_launch(description)
        except GLib.Error as exc:
            raise RuntimeError(f"Invalid GStreamer pipeline: {exc}") from exc

        sink = pipeline.get_by_name("ros_sink")
        if sink is None:
            raise RuntimeError(
                "The pipeline has no element named 'ros_sink'; the driver needs an "
                "appsink with that name to pull frames from."
            )
        sink.connect("new-sample", self._on_new_sample)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)

        pipeline.set_state(Gst.State.PLAYING)
        self._pipeline = pipeline
        self._last_frame_time = self.get_clock().now()

    def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

    def _on_bus_error(self, _bus, message) -> None:
        error, debug = message.parse_error()
        self.get_logger().error(f"GStreamer error: {error.message} ({debug})")

    def _on_bus_warning(self, _bus, message) -> None:
        warning, debug = message.parse_warning()
        self.get_logger().warning(f"GStreamer warning: {warning.message} ({debug})")

    # -- Frame handling -------------------------------------------------------------

    def _on_new_sample(self, sink) -> int:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")

        success, mapped = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            # np.frombuffer gives a read-only view into GStreamer-owned memory that is
            # unmapped as soon as this callback returns, so copy before publishing.
            frame = np.frombuffer(mapped.data, dtype=np.uint8)
            frame = frame.reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(mapped)

        self._publish_frame(frame, width, height)
        return Gst.FlowReturn.OK

    def _publish_frame(self, frame: np.ndarray, width: int, height: int) -> None:
        now = self.get_clock().now()
        stamp = rclpy.time.Time(
            nanoseconds=now.nanoseconds + self._timestamp_offset_ns
        ).to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = width * 3
        # rosidl only fast-paths array.array('B'). Assigning bytes/ndarray falls into a
        # per-element `all(isinstance(v, int) ...)` validation loop over every byte:
        # 322 ms for a 1080p BGR frame, i.e. a 3 fps ceiling. This is ~95x faster and
        # produces byte-identical payloads.
        image.data = array.array('B', frame.tobytes())
        self._pub_image.publish(image)

        info = self._camera_info
        if info is None:
            info = uncalibrated_camera_info(width, height, self._frame_id)
            self._camera_info = info
        elif info.width != width or info.height != height:
            self.get_logger().warning(
                f"Calibration is for {info.width}x{info.height} but the stream is "
                f"{width}x{height}; the intrinsics do not apply to these frames",
                throttle_duration_sec=30.0,
            )
        info.header.stamp = stamp
        self._pub_info.publish(info)

        if self._publish_compressed:
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
            if ok:
                compressed = CompressedImage()
                compressed.header.stamp = stamp
                compressed.header.frame_id = self._frame_id
                compressed.format = "jpeg"
                compressed.data = array.array('B', encoded.tobytes())
                self._pub_compressed.publish(compressed)

        with self._counter_lock:
            self._frame_count += 1
            self._frames_since_report += 1
        self._last_frame_time = now

    # -- Watchdog -------------------------------------------------------------------

    def _check_stream(self) -> None:
        period = float(self.get_parameter("watchdog_period_s").value)
        with self._counter_lock:
            frames = self._frames_since_report
            self._frames_since_report = 0

        if frames > 0:
            self.get_logger().info(f"camera {frames / period:.1f} fps")
            return

        silence = (self.get_clock().now() - self._last_frame_time).nanoseconds / 1e9
        limit = float(self.get_parameter("restart_after_silence_s").value)
        self.get_logger().warning(
            f"No frames for {silence:.1f} s. Check that the ROV is streaming and that "
            f"UDP port {self.get_parameter('udp_port').value} reaches this host."
        )
        if silence > limit:
            self.get_logger().warning("Restarting the GStreamer pipeline")
            self._stop_pipeline()
            try:
                self._start_pipeline()
            except RuntimeError as exc:
                self.get_logger().error(f"Pipeline restart failed: {exc}")

    def destroy_node(self) -> bool:
        self._stop_pipeline()
        if self._loop.is_running():
            self._loop.quit()
        if self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueRov2CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
