#!/usr/bin/env python3
"""Stream the two aux cameras of the multi-camera system, stamped with their capture time.

Runs ON the camera computer (NVIDIA Jetson). Do not start it by hand: `multicam sync-start`
copies it to ~/rov4_vslam/ there, stops the Blue Atlas camera service (which holds every
camera open) and starts this script; `multicam sync-stop` / `multicam shutdown` stop it.
Nothing here is enabled at boot, so a power cycle always brings the Blue Atlas software
back exactly as it was.

    python3 aux_sync_pipeline.py HOST [PORT_LEFT] [PORT_RIGHT] [BITRATE]

Why not the Blue Atlas stream (measured 2026-10-30): its pipeline pushes every camera into
an intervideosink and re-times it to a 30 Hz tick of the Jetson clock (intervideosrc). The
cameras, driven by frame_sync, run at 30.0028 Hz (+94 ppm), so the phase between frames and
tick slips by a whole frame every ~355 s. Then that camera repeats or drops a frame, at a
different time for each camera, and the left / right pairing can end up shifted by one
frame. The RTP timestamps come from the tick, so nothing on the topside can tell.

Here both cameras go through ONE pipeline. v4l2src stamps each buffer with the kernel
capture time of the frame and nothing re-times the frames. The two aux cameras capture
within +-14 us of each other (kernel timestamps, 900 frames, 2026-10-30).

The capture time is sent as wall-clock time. The pipeline runs on the system REALTIME
clock with base time 0, so a buffer's running time is the wall-clock time of its capture
(v4l2src converts the kernel's monotonic capture time to the pipeline clock), and
rtph265pay with timestamp-offset=0 writes it into the RTP timestamp, at 90 kHz and modulo
2^32 ticks (~13.3 h). The topside camera node (timestamp_source: "capture") turns it back
into wall-clock time, picking the 13.3-hour period from the arrival time. That is the
capture time on the topside clock as long as this computer's clock is synchronised to the
topside (`multicam timesync-setup`, once): no latency estimate is involved.

The image matches the Blue Atlas stream (checked side by side): the full 1920x1080 sensor
image scaled to 960x540, H.265 from the hardware encoder at 3 Mbit/s, RTP payload 96.
"""

import signal
import subprocess
import sys

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, GObject, Gst  # noqa: E402


def branch(camera: str, host: str, port: int, bitrate: int) -> str:
    return (
        f"v4l2src device=/dev/video_{camera} "
        "! video/x-raw,format=UYVY,width=1920,height=1080 "
        "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=960,height=540 "
        f"! nvv4l2h265enc bitrate={bitrate} maxperf-enable=1 insert-sps-pps=true "
        "iframeinterval=30 idrinterval=30 "
        "! h265parse ! rtph265pay config-interval=1 pt=96 timestamp-offset=0 "
        f"! udpsink host={host} port={port} sync=false"
    )


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    host = sys.argv[1]
    port_left = int(sys.argv[2]) if len(sys.argv) > 2 else 5700
    port_right = int(sys.argv[3]) if len(sys.argv) > 3 else 5701
    bitrate = int(sys.argv[4]) if len(sys.argv) > 4 else 3000000

    # The Blue Atlas service applies its camera settings when it starts and they stay
    # set, but hardware frame sync is what makes the pair capture together.
    for camera in ("aux_left", "aux_right"):
        subprocess.run(["v4l2-ctl", "-d", f"/dev/video_{camera}", "-c", "frame_sync=1"],
                       check=True)

    Gst.init(None)
    pipeline = Gst.parse_launch(
        branch("aux_left", host, port_left, bitrate) + " "
        + branch("aux_right", host, port_right, bitrate)
    )
    # Running time = wall-clock time: REALTIME clock, base time 0, and no start time so
    # that the pipeline does not set its own base time when it goes to PLAYING.
    clock = GObject.new(Gst.SystemClock, clock_type=Gst.ClockType.REALTIME)
    pipeline.use_clock(clock)
    pipeline.set_start_time(Gst.CLOCK_TIME_NONE)
    pipeline.set_base_time(0)

    loop = GLib.MainLoop()
    result = {"code": 0}

    def on_message(_bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            print(f"ERROR: {error.message} ({debug})", flush=True)
            result["code"] = 1
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message)

    def stop(*_args):
        # End of stream first, so the encoders and payloaders finish cleanly; give up
        # after 3 s.
        pipeline.send_event(Gst.Event.new_eos())
        GLib.timeout_add_seconds(3, loop.quit)
        return GLib.SOURCE_REMOVE

    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, stop)
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, stop)

    if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        print("ERROR: the pipeline does not start", flush=True)
        return 1
    print(f"streaming aux_left -> {host}:{port_left}, aux_right -> {host}:{port_right}, "
          "RTP timestamps = capture wall-clock time", flush=True)
    loop.run()
    pipeline.set_state(Gst.State.NULL)
    return result["code"]


if __name__ == "__main__":
    sys.exit(main())
