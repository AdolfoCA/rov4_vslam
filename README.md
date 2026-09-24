# BlueROV2 Heavy — ROS 2 Humble sensor stack

A containerised ROS 2 Humble workspace that streams the sensors of a BlueROV2 Heavy onto
ROS topics: the **Navigator IMU**, a **Water Linked DVL-A50**, the **monocular low-light
camera**, and the four cameras of the **Blue Atlas multi-camera system** mounted on the
vehicle. Drivers and recording only — no control, no estimation.

> **First time against a vehicle?** Follow **[TEST_PLAN.md](TEST_PLAN.md)** — a staged
> bring-up procedure with pass criteria, expected numbers and a results sheet. For what
> every topside laptop needs (network addresses, kernel buffer limit), see
> **[TOPSIDE_SETUP.md](TOPSIDE_SETUP.md)**.

---

## 1. How the data actually gets here

This is worth being explicit about, because it explains why each driver looks the way it
does. Nothing on the ROV runs this ROS stack. BlueOS on the ROV's Raspberry Pi owns the
vehicle hardware and exposes each sensor over the tether in its own way:

| Sensor | Who owns it on the ROV | How it reaches topside | Driver strategy |
|---|---|---|---|
| IMU (ICM-20602 + MMC5983 + BMP280 on the Navigator) | ArduSub | MAVLink, UDP :14550 | `pymavlink` client that requests message intervals |
| DVL-A50 | the instrument itself | its own TCP server, :16171 | TCP client parsing newline-delimited JSON |
| Low-light (nose) camera | BlueOS video pipeline | H.264 over RTP/UDP, :5600 | GStreamer `appsink` pipeline |
| Multi-camera system, 4 cameras | its own NVIDIA computer (Blue Atlas Robotics) | H.265 over RTP/UDP to `192.168.1.1`, :5700–5703 | the same camera driver, one node per camera, `codec: h265` |

Four different transports, four different failure modes. Every driver therefore runs
its I/O on its own thread, reconnects on its own, and reports its observed rate every
five seconds so a silent sensor is obvious in the log rather than invisible.

The multi-camera system is a separate computer on the ROV's network (`10.42.0.5` and
`192.168.1.4`), with its own battery. It runs its own ROS 2 nodes under `/ba/...`
(CycloneDDS) to start and stop streams and recordings and to set its lights; its message
definitions are in `ros2_ws/src/ba_msgs`. The drivers here only **receive** its video:
the camera system must already be streaming.

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

By default this starts the IMU, DVL and nose camera drivers, the static TF node, the
Foxglove bridge, and the **top pair** of the multi-camera system (Aux Left, Aux Right).

Selective bring-up and overrides:

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py camera:=false     # no nose camera
ros2 launch bluerov2_bringup bluerov2.launch.py stereo_bottom:=true bottom_most:=true   # all 4 multicams
ros2 launch bluerov2_bringup bluerov2.launch.py aux_left:=false aux_right:=false        # no multicams
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

## 3. Recording

Record with `record.launch.py`, in a **second shell** inside the container while the
drivers run. Stop with **Ctrl-C**; the bag is closed cleanly.

```bash
docker compose exec bluerov2 bash

# 1. is everything I want to record actually publishing?
ros2 launch bluerov2_bringup record.launch.py check_only:=true

# 2. record (the same check runs first; recording starts only if it passes)
ros2 launch bluerov2_bringup record.launch.py bag_name:=dive01
```

Every sensor has its own switch. **Defaults: IMU, DVL and the multi-camera top pair on;
nose camera and bottom pair off.**

```bash
ros2 launch bluerov2_bringup record.launch.py camera:=true                        # + nose camera
ros2 launch bluerov2_bringup record.launch.py camera:=true camera_raw:=false      # + nose camera, JPEG only
ros2 launch bluerov2_bringup record.launch.py stereo_bottom:=true bottom_most:=true   # + bottom pair
ros2 launch bluerov2_bringup record.launch.py dvl:=false                          # no DVL
ros2 launch bluerov2_bringup record.launch.py multicam_raw:=true                  # + uncompressed multicam frames
```

| Argument | Default | Records |
|---|---|---|
| `imu` | `true` | `imu/data`, `imu/data_raw`, `imu/mag`, `imu/pressure`, `imu/temperature` |
| `dvl` | `true` | `dvl/velocity`, `dvl/report`, `dvl/altitude`, `dvl/dead_reckoning`, `dvl/dead_reckoning_odometry` |
| `camera` | `false` | nose camera `camera/camera_info` + the two below |
| `camera_raw` | `true` | nose `camera/image_raw` (1080p, ~187 MB/s) |
| `camera_compressed` | `true` | nose `camera/image_raw/compressed` (JPEG) |
| `aux_left`, `aux_right` | `true` | multi-camera top pair |
| `stereo_bottom`, `bottom_most` | `false` | multi-camera bottom pair |
| `multicam_raw` | `false` | `multicam/<camera>/image_raw` (960×540, ~39 MB/s per camera) |
| `multicam_compressed` | `true` | `multicam/<camera>/image_raw/compressed` (JPEG) |
| `tf` / `logs` | `true` | `/tf`, `/tf_static` / `/rosout` |
| `check` | `true` | check the selected streams before recording |
| `check_only` | `false` | only run the check, do not record |
| `check_timeout` | `10` | seconds to wait for the first message of each stream |
| `bag_name` | `rov_YYYYmmdd_HHMMSS` | one folder per recording, in `rov4_vslam/data/` |
| `max_bag_duration` | `60` | split the bag every N seconds (0 = one file) |
| `storage` | `mcap` | or `sqlite3` |

**The stream check.** Before recording, `check_streams.py` subscribes to each selected
sensor, waits up to `check_timeout` for data and measures the rate. It prints a table
like this:

```
STREAM          TOPIC                                          RATE     MIN  RESULT
imu             /bluerov2/imu/data_raw                     102.6 Hz     50  OK
dvl             /bluerov2/dvl/report                         4.0 Hz      1  OK
aux_left        /bluerov2/multicam/aux_left/camera_info     29.6 Hz     10  OK
aux_right       /bluerov2/multicam/aux_right/camera_info    30.3 Hz     10  OK

STREAM CHECK PASSED: all 4 streams are publishing
```

If any selected sensor is silent or below its minimum rate, it prints `STREAM CHECK
FAILED: <sensor>` and **no recording is started** (no bag folder is created). Either
start that sensor or switch it off. `check:=false` skips the check.

A multi-camera that should be recorded must also be **running**: turn it on in
`bluerov2.launch.py` too (`stereo_bottom:=true` etc.), otherwise the check reports it
missing.

**Disk.** Raw nose-camera video fills about 11 GB per minute; a 26 s test with every
default of the old launch file was 5 GB. The free space is printed when recording starts.

## 4. Topics

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
| `camera/image_raw` | `sensor_msgs/Image` | 30 fps | `bgr8`, nose camera |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | 30 fps | zeros until calibrated |
| `camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 fps | JPEG; on when the Foxglove bridge runs |
| `multicam/<camera>/image_raw` | `sensor_msgs/Image` | 30 fps | `bgr8`, 960×540 |
| `multicam/<camera>/camera_info` | `sensor_msgs/CameraInfo` | 30 fps | zeros until calibrated |
| `multicam/<camera>/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 fps | JPEG, always on |

`<camera>` is `aux_left`, `aux_right`, `stereo_bottom` or `bottom_most`. Ports, codec and
frame ids per camera are in `config/bluerov2.yaml`:

| Camera | UDP port | Position (seen from the front) |
|---|---|---|
| `aux_right` (cam0) | 5701 | top left |
| `aux_left` (cam1) | 5700 | top right, 0.22 m from aux_right |
| `bottom_most` (cam2) | 5703 | bottom left, 0.11 m below aux_right |
| `stereo_bottom` (cam3) | 5702 | bottom right, 0.22 m from bottom_most |

The DVL rate is bounded by acoustics, not software: the instrument cannot ping again
until the previous ping returns, so the rate falls as altitude rises.

## 5. Foxglove

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
take a minute to add by hand; the topic paths above are the ones you want. For the
multi-cameras, add an Image panel on `multicam/<camera>/image_raw/compressed`.

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

## 6. Frames

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

The multi-camera frames (`multicam_<camera>_optical`) are set on the images but are
**not yet in the TF tree**: their mounting has not been measured.

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

## 7. Timestamps

The IMU driver does not stamp on arrival. It estimates the offset between the vehicle
clock and the ROS clock with a sliding-window minimum filter (`bluerov2_imu.time_sync`)
and maps vehicle timestamps onto ROS time. The reasoning: every offset sample is
inflated by that message's transport delay, so the smallest offset in a recent window
is the least-delayed sample — the best passive estimate available. This removes tether
jitter from the stamps, which matters far more to a filter than a constant bias does.

The cameras cannot do this. Their RTP timestamps have no shared epoch and there are no
RTCP sender reports to anchor them, so frames are stamped on arrival and the encode +
tether + decode latency is baked in. `timestamp_offset_s` (per camera) exists so a
measured offset can be applied without touching the code.

All sensors are stamped on the same ROS clock and recorded into the same bag, so they
can be aligned by `header.stamp`. The four multi-camera streams are **not synchronised
with each other** by the camera system (its streaming mode is explicitly
unsynchronised), so a stereo pair's frames have close but not identical times.

## 8. Before the first dive — the TODO list

Every item below is marked `TODO` in the source. The first group will stop the stack
from working; the second will let it work while producing quietly wrong numbers, which
is worse.

**Will not work until set**

1. **`ROV_IP` / `DVL_IP`** — `docker-compose.yaml`. Defaults are the BlueROV2 factory
   values (`192.168.2.2`, `192.168.2.95`). Confirm with `ping`. On the lab vehicle the
   DVL is at `192.168.2.128`.
2. **MAVLink endpoint** — `connection_url` in `config/bluerov2.yaml`. The default
   `udpin:0.0.0.0:14550` listens for what BlueOS pushes to topside. If your BlueOS
   expects a client instead, use `udpout:<rov-ip>:14550`.
3. **Video port** — `udp_port`, default 5600. The BlueOS video page decides this.
4. **Topside network** — the adapter facing the tether needs **three** fixed addresses,
   and no gateway: `192.168.2.1/24` (BlueOS sends MAVLink and video here),
   `192.168.1.1/24` (the multi-camera system sends its video here) and `10.42.0.1/24`
   (the multi-camera computer's network; it uses `10.42.0.1` as its gateway). Plus
   `net.core.rmem_max=67108864` for raw video. Exact commands in
   [TOPSIDE_SETUP.md](TOPSIDE_SETUP.md).
5. **Multi-camera streaming** — the camera system must be streaming to `192.168.1.1` on
   ports 5700–5703. It is started on the camera system with its
   `/ba/camera_feed_manager/new_attach_streaming_consumer` service. Whether it resumes
   by itself after a restart is still to be checked.

**Will work, but produce wrong numbers**

6. **Sensor extrinsics** — `config/extrinsics.yaml`. A DVL lever arm off by 5 cm
   injects a phantom velocity of `omega x r`: at 20 °/s of yaw that is ~1.7 cm/s of
   sway, which any estimator will happily integrate into drift while nothing looks
   broken. Start from CAD, refine on real data. `static_tf_node` warns about every
   transform still left at identity. The multi-cameras are not in the tree yet.
7. **Camera calibration** — `config/camera_info.yaml` is a *placeholder*, not a
   calibration, and the multi-cameras have none. Calibrate **in water**: a flat port
   scales the effective focal length by roughly the refractive index of water (≈1.33)
   and adds distortion no in-air calibration captures.
8. **IMU noise parameters** — the `*_stddev` values are order-of-magnitude guesses.
   Run an Allan variance on your own unit before any estimator trusts them.
9. **DVL fallback covariance** — only used when the instrument sends neither a
   covariance nor a usable figure of merit.

## 9. Layout

```
rov4_vslam/
├── Dockerfile                     ROS 2 Humble + GStreamer + pymavlink
├── docker-compose.yaml            host networking, bind-mounted source, TODO IPs
├── requirements.txt
├── rtps_udp_profile.xml           Fast DDS: plain UDPv4, 32 MB receive buffer
├── TOPSIDE_SETUP.md               what every topside laptop needs
└── ros2_ws/src/
    ├── start_mission.sh
    ├── bluerov2_msgs/             DVLReport, DVLBeam, DVLDeadReckoning
    ├── ba_msgs/                   multi-camera system services (Blue Atlas Robotics)
    ├── bluerov2_imu/              MAVLink driver + frames.py + time_sync.py
    ├── bluerov2_dvl/              TCP JSON driver + protocol.py
    ├── bluerov2_camera/           GStreamer driver (H.264 / H.265) + calibration.py
    ├── bluerov2_tf/               static TF node + extrinsics.py (parse & validate)
    └── bluerov2_bringup/          launch and configuration
        ├── launch/bluerov2.launch.py    drivers + multicams + static TF + Foxglove bridge
        ├── launch/record.launch.py      stream check + rosbag recording
        ├── launch/foxglove.launch.py    the bridge on its own
        ├── scripts/check_streams.py     the stream check used before recording
        ├── config/bluerov2.yaml         driver parameters (incl. the 4 multicams)
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

## 10. Troubleshooting

| Symptom | Where to look |
|---|---|
| `No MAVLink heartbeat within 10 s` | `ping $ROV_IP`; `tcpdump -i any udp port 14550`; check BlueOS endpoints |
| IMU connects but topics are slow | ArduSub refused the interval request; watch the 5 s rate log |
| `No DVL reports in the last 5 s` | `nc -vz $DVL_IP 16171`; the DVL's own web UI |
| DVL velocity topic silent, `dvl/report` alive | no bottom lock — expected out of range or over soft mud |
| No frames | `gst-launch-1.0 udpsrc port=5600 ! fakesink -v` inside the container |
| Video tears or stutters | raise `jitter_buffer_ms` |
| Raw frames at 1–3 fps in a bag or another node | `net.core.rmem_max` not raised on the host, see TOPSIDE_SETUP.md |
| `STREAM CHECK FAILED` before recording | the named sensor is not publishing: start it, or switch it off in `record.launch.py` |
| Multi-camera node logs `No frames` | the camera system is not streaming, or the host does not own `192.168.1.1`; `tcpdump -i any -c 5 udp portrange 5700-5703` |
| Multi-camera computer not answering | `ping -c3 10.42.0.5` must show `ttl=64`; replies with a lower ttl come from another machine over Wi-Fi |
| `/ba/...` service calls never answer | the camera system uses CycloneDDS; calling its services from Fast DDS does not work. Topics do |
| TF lookups fail or give odd answers | `ros2 run tf2_tools view_frames`; check the `static_tf_node` startup log for warnings |
| Nodes cannot see each other | check `ROS_DOMAIN_ID` and that `FASTRTPS_DEFAULT_PROFILES_FILE` points at a valid file |
| Foxglove will not connect | `ss -ltnp \| grep 8765` in the container; check the host firewall; confirm the URL is `ws://` not `http://` |
| Foxglove connects, then drops during video | you are subscribed to raw `image_raw` — switch the Image panel to `image_raw/compressed` |
| Foxglove sees no topics | the bridge is up but the drivers are not; check `ros2 topic list` |
