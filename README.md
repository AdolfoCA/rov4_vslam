# BlueROV2 Heavy — ROS 2 Humble sensor stack

A containerised ROS 2 Humble workspace that streams the three sensors of a BlueROV2
Heavy onto ROS topics: the **Navigator IMU**, a **Water Linked DVL-A50**, and the
**monocular low-light camera**. Drivers only — no control, no estimation.

> **First time against a vehicle?** Follow **[TEST_PLAN.md](TEST_PLAN.md)** — a staged
> bring-up procedure with pass criteria, expected numbers and a results sheet. This code
> has never run against hardware, so start there rather than here.

---

## 1. How the data actually gets here

This is worth being explicit about, because it explains why each driver looks the way it
does. Nothing on the ROV runs ROS. BlueOS on the ROV's Raspberry Pi owns the hardware
and exposes each sensor over the tether in its own way:

| Sensor | Who owns it on the ROV | How it reaches topside | Driver strategy |
|---|---|---|---|
| IMU (ICM-20602 + MMC5983 + BMP280 on the Navigator) | ArduSub | MAVLink, UDP :14550 | `pymavlink` client that requests message intervals |
| DVL-A50 | the instrument itself | its own TCP server, :16171 | TCP client parsing newline-delimited JSON |
| Low-light camera | BlueOS video pipeline | H.264 over RTP/UDP, :5600 | GStreamer `appsink` pipeline |

Three different transports, three different failure modes. Every driver therefore runs
its I/O on its own thread, reconnects on its own, and reports its observed rate every
five seconds so a silent sensor is obvious in the log rather than invisible.

## 2. Quick start

```bash
git clone https://github.com/AdolfoCA/rov4_vslam.git
cd rov4_vslam
docker compose build            # first time, ~10 min
docker compose up -d
docker compose exec bluerov2 bash

# inside the container
ros2 launch bluerov2_bringup bluerov2.launch.py
```

Selective bring-up and overrides:

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py camera:=false     # IMU + DVL only
ros2 launch bluerov2_bringup bluerov2.launch.py dvl_ip:=192.168.2.95
ros2 launch bluerov2_bringup bluerov2.launch.py foxglove:=false   # no websocket
```

Or use the tmux helper, which runs the drivers, a rate monitor and an optional
recording in separate windows:

```bash
start_mission            # drivers only
start_mission --record   # + an MCAP rosbag into ./data
stop_mission
```

After editing source on the host, rebuild inside the container with `build_ws` — the
workspace source is bind-mounted, so no image rebuild is needed.

## 3. Topics

All under the `/bluerov2` namespace by default.

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | 50 Hz | orientation + rates + accel |
| `imu/data_raw` | `sensor_msgs/Imu` | 100 Hz | no orientation (`orientation_covariance[0] = -1`) |
| `imu/mag` | `sensor_msgs/MagneticField` | 100 Hz | |
| `imu/pressure` | `sensor_msgs/FluidPressure` | 10 Hz | **hull** pressure, not depth |
| `imu/temperature` | `sensor_msgs/Temperature` | 100 Hz | only if the IMU reports one |
| `dvl/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | ≤15 Hz | bottom-track |
| `dvl/report` | `bluerov2_msgs/DVLReport` | ≤15 Hz | per-beam, FOM, altitude |
| `dvl/altitude` | `sensor_msgs/Range` | ≤15 Hz | |
| `dvl/dead_reckoning` | `bluerov2_msgs/DVLDeadReckoning` | ~5 Hz | instrument's own integration |
| `dvl/dead_reckoning_odometry` | `nav_msgs/Odometry` | ~5 Hz | same, as odometry |
| `camera/image_raw` | `sensor_msgs/Image` | 30 fps | `bgr8` |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | 30 fps | zeros until calibrated |
| `camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 fps | JPEG; on when the Foxglove bridge runs |

The DVL rate is bounded by acoustics, not software: the instrument cannot ping again
until the previous ping returns, so the rate falls as altitude rises.

## 4. Foxglove

`foxglove_bridge` starts with the main launch by default. Point Foxglove — the desktop
app, or the web app at app.foxglove.dev — at:

```
ws://<host running the container>:8765
```

The container uses host networking, so that is this machine's own address and no port
mapping is involved. Foxglove itself does not need ROS installed, and no DDS traffic
leaves the machine: the bridge multiplexes the entire graph over that one websocket.
This matters on a boat, where DDS multicast discovery across a tether or a shared lab
network is one of the more reliable ways to lose an afternoon.

A prepared layout is at `ros2_ws/src/bluerov2_bringup/config/foxglove_layout.json` —
3D scene with the frame tree, camera image, IMU rate and acceleration plots, DVL
velocity, and an altitude/FOM plot for watching bottom lock. Import it with
**Layout → Import from file**. If your Foxglove version rejects part of it, the panels
take a minute to add by hand; the topic paths above are the ones you want.

### Video bandwidth

Raw `bgr8` at 1080p30 is about **186 MB/s**. No websocket will carry that, and Foxglove
will stall and then be dropped by the bridge's send buffer. The same frames as JPEG are
about **5 MB/s**, which is fine.

So the launch file turns JPEG republishing on automatically whenever the bridge is
running (`compressed_video:=auto`, the default) and you should point Foxglove's Image
panel at `camera/image_raw/compressed`, which the prepared layout already does. The
raw topic stays available for local consumers — a SLAM front end in the same container
pays no encode cost and reads `image_raw` directly.

Override with `compressed_video:=true|false` if you want to decide yourself.

### Bridge configuration

`config/foxglove_bridge.yaml`. Two settings worth knowing about:

- **`capabilities`** deliberately omits `clientPublish`. Nothing in this stack should be
  commandable from a visualisation tool; add it only if you decide otherwise.
- **`use_compression`** is off. Websocket deflate on JPEG or H.264 payloads burns CPU
  for nothing, and the numeric topics are too small to care. Turn it on only for a
  genuinely narrow tunnel with CPU to spare.

To run the bridge without the drivers:

```bash
ros2 launch bluerov2_bringup foxglove.launch.py port:=8765
```

## 5. Frames

```
base_link ──> imu_link
          ──> dvl_link
          ──> camera_link ──> camera_link_optical
```

Everything is converted into REP-103 conventions at the driver boundary, so nothing
downstream has to think about it:

- **ArduPilot** reports body-frame data in FRD and attitude relative to NED. Converted
  to FLU / ENU by `bluerov2_imu.frames`.
- **The DVL-A50** reports in FRD. Converted to FLU by `bluerov2_dvl.protocol`
  (`rotate_to_flu`, on by default).
- **`camera_link_optical`** follows the vision convention (z forward); the fixed
  rotation from `camera_link` is the standard `(-π/2, 0, -π/2)` and should not be
  changed.

The extrinsics themselves live in `config/extrinsics.yaml` and are published by
`bluerov2_tf`'s `static_tf_node` — one node reading one file, rather than a
`static_transform_publisher` process per transform with the numbers buried in launch
arguments. That buys three things:

- **Validation at startup.** A frame given two parents, a cycle, a non-unit quaternion,
  a bad translation — each is caught with a message naming the offending transform, and
  the node refuses to publish rather than emitting a partly-wrong tree. A frame with two
  parents is the nastiest of these: tf2 accepts both and then resolves lookups according
  to whichever message arrived last.
- **Degrees.** Set `units: degrees` per transform and write `-90` instead of
  `-1.5707963`. Mounting angles come off drawings in degrees.
- **A nag for placeholders.** Any transform left at exact identity is warned about at
  startup, so a forgotten measurement shows up in the log rather than in your results.

Rotations take either `rpy: [roll, pitch, yaw]` (intrinsic Z-Y-X, the ROS convention) or
`quaternion: [x, y, z, w]` for values that came out of a calibration routine.

The shipped values are **placeholders**. Turn the node off with `static_tf:=false` if
something else in your system owns the tree, or point it elsewhere with
`extrinsics_file:=<path>`.

## 6. Timestamps

The IMU driver does not stamp on arrival. It estimates the offset between the vehicle
clock and the ROS clock with a sliding-window minimum filter (`bluerov2_imu.time_sync`)
and maps vehicle timestamps onto ROS time. The reasoning: every offset sample is
inflated by that message's transport delay, so the smallest offset in a recent window
is the least-delayed sample — the best passive estimate available. This removes tether
jitter from the stamps, which matters far more to a filter than a constant bias does.

The camera cannot do this. Its RTP timestamps have no shared epoch and there are no
RTCP sender reports to anchor them, so frames are stamped on arrival and the encode +
tether + decode latency is baked in. `timestamp_offset_s` exists so a measured offset
can be applied without touching the code.

## 7. Before the first dive — the TODO list

Every item below is marked `TODO` in the source. The first group will stop the stack
from working; the second will let it work while producing quietly wrong numbers, which
is worse.

**Will not work until set**

1. **`ROV_IP` / `DVL_IP`** — `docker-compose.yaml`. Defaults are the BlueROV2 factory
   values (`192.168.2.2`, `192.168.2.95`). Confirm with `ping`.
2. **MAVLink endpoint** — `connection_url` in `config/bluerov2.yaml`. The default
   `udpin:0.0.0.0:14550` listens for what BlueOS pushes to topside. If your BlueOS
   expects a client instead, use `udpout:<rov-ip>:14550`.
3. **Video port** — `udp_port`, default 5600. The BlueOS video page decides this.
4. **Topside network** — this host normally needs `192.168.2.1/24` on the interface
   facing the tether interface board.

**Will work, but produce wrong numbers**

5. **Sensor extrinsics** — `config/extrinsics.yaml`. A DVL lever arm off by 5 cm
   injects a phantom velocity of `omega x r`: at 20 °/s of yaw that is ~1.7 cm/s of
   sway, which any estimator will happily integrate into drift while nothing looks
   broken. Start from CAD, refine on real data. `static_tf_node` warns about every
   transform still left at identity.
6. **Camera calibration** — `config/camera_info.yaml` is a *placeholder*, not a
   calibration. Calibrate **in water**: a flat port scales the effective focal length by
   roughly the refractive index of water (≈1.33) and adds distortion no in-air
   calibration captures.
7. **IMU noise parameters** — the `*_stddev` values are order-of-magnitude guesses.
   Run an Allan variance on your own unit before any estimator trusts them.
8. **DVL fallback covariance** — only used when the instrument sends neither a
   covariance nor a usable figure of merit.

## 8. Layout

```
rov4_vslam/
├── Dockerfile                     ROS 2 Humble + GStreamer + pymavlink
├── docker-compose.yaml            host networking, bind-mounted source, TODO IPs
├── requirements.txt
├── rtps_udp_profile.xml           Fast DDS: plain UDPv4, no shared memory
└── ros2_ws/src/
    ├── start_mission.sh
    ├── bluerov2_msgs/             DVLReport, DVLBeam, DVLDeadReckoning
    ├── bluerov2_imu/              MAVLink driver + frames.py + time_sync.py
    ├── bluerov2_dvl/              TCP JSON driver + protocol.py
    ├── bluerov2_camera/           GStreamer driver + calibration.py
    ├── bluerov2_tf/               static TF node + extrinsics.py (parse & validate)
    └── bluerov2_bringup/          launch and configuration
        ├── launch/bluerov2.launch.py    drivers + static TF + Foxglove bridge
        ├── launch/foxglove.launch.py    the bridge on its own
        ├── config/bluerov2.yaml         driver parameters
        ├── config/extrinsics.yaml       sensor mounting — measure these
        ├── config/foxglove_bridge.yaml  bridge parameters
        ├── config/foxglove_layout.json  importable Foxglove layout
        └── config/camera_info.yaml      placeholder calibration
```

Each driver keeps its pure logic — frame conversions, clock offsets, wire decoding — in
a module with no ROS dependencies, so it can be tested without a ROS graph or hardware:

```bash
colcon test --packages-select bluerov2_imu bluerov2_dvl bluerov2_tf \
  && colcon test-result --verbose
```

60 tests cover the frame rotations (against hand-derived cases), the clock estimator,
the DVL JSON parsing including malformed input and split TCP reads, the MAVLink unit
scalings (via real encode/decode round-trips), and the extrinsics parsing and TF tree
validation.

## 9. Troubleshooting

| Symptom | Where to look |
|---|---|
| `No MAVLink heartbeat within 10 s` | `ping $ROV_IP`; `tcpdump -i any udp port 14550`; check BlueOS endpoints |
| IMU connects but topics are slow | ArduSub refused the interval request; watch the 5 s rate log |
| `No DVL reports in the last 5 s` | `nc -vz $DVL_IP 16171`; the DVL's own web UI |
| DVL velocity topic silent, `dvl/report` alive | no bottom lock — expected out of range or over soft mud |
| No frames | `gst-launch-1.0 udpsrc port=5600 ! fakesink -v` inside the container |
| Video tears or stutters | raise `jitter_buffer_ms` |
| TF lookups fail or give odd answers | `ros2 run tf2_tools view_frames`; check the `static_tf_node` startup log for warnings |
| Nodes cannot see each other | check `ROS_DOMAIN_ID` and that `FASTRTPS_DEFAULT_PROFILES_FILE` points at a valid file |
| Foxglove will not connect | `ss -ltnp \| grep 8765` in the container; check the host firewall; confirm the URL is `ws://` not `http://` |
| Foxglove connects, then drops during video | you are subscribed to raw `image_raw` — switch the Image panel to `image_raw/compressed` |
| Foxglove sees no topics | the bridge is up but the drivers are not; check `ros2 topic list` |
