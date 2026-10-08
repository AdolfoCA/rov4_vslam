# BlueROV2 Heavy — ROS 2 Humble sensor stack

A containerised ROS 2 Humble workspace that streams the sensors of a BlueROV2 Heavy onto
ROS topics: the **Navigator IMU**, a **Water Linked DVL-A50**, the **monocular low-light
camera**, and the four cameras of the **Blue Atlas multi-camera system** mounted on the
vehicle. Drivers, recording and session control — no vehicle control. One estimator is
included: **AQUA-SLAM** (stereo + DVL + gyro SLAM), section 8b.

> **New laptop?** Do the one-time setup in **section 3** first (network addresses, kernel
> buffer, the multi-camera password file). Details and background:
> **[TOPSIDE_SETUP.md](TOPSIDE_SETUP.md)**. First time against a new vehicle:
> **[TEST_PLAN.md](TEST_PLAN.md)**.

---

## 1. How the data actually gets here

This is worth being explicit about, because it explains why each driver looks the way it
does. Nothing on the ROV runs this ROS stack. BlueOS on the ROV's Raspberry Pi owns the
vehicle hardware and exposes each sensor over the tether in its own way:

| Sensor | Who owns it on the ROV | How it reaches topside | Driver strategy |
|---|---|---|---|
| IMU (ICM-20602 + MMC5983 + BMP280 on the Navigator) | ArduSub | MAVLink, UDP :14551 (a copy of :14550) | `pymavlink` client that requests message intervals |
| DVL-A50 | the instrument itself | its own TCP server, `192.168.2.128`:16171 | TCP client parsing newline-delimited JSON |
| Low-light (nose) camera | BlueOS video pipeline | H.264 over RTP/UDP, :5600 (relayed to :5602) | GStreamer `appsink` pipeline |
| Multi-camera system, 4 cameras | its own NVIDIA computer (Blue Atlas Robotics) | H.265 over RTP/UDP to `192.168.1.1`, :5700–5703 | the same camera driver, one node per camera, `codec: h265` |

Four different transports, four different failure modes. Every driver therefore runs
its I/O on its own thread, reconnects on its own, and reports its observed rate every
five seconds so a silent sensor is obvious in the log rather than invisible.

The multi-camera system is a separate computer on the ROV's network (`10.42.0.5` and
`192.168.1.4`, fixed), with its own battery. It runs its own ROS 2 nodes under `/ba/...`
on **CycloneDDS**: they start and stop the video streams and recordings and set the
lights. Its message definitions are in `ros2_ws/src/ba_msgs`, and the `multicam` command
(section 5) calls them.

**Works next to QGroundControl.** QGroundControl on the same laptop keeps UDP 14550
(MAVLink) and 5600 (video) unchanged: the IMU driver listens on 14551, where BlueOS
already sends an identical copy, and a relay copies the nose-camera video to QGC and to
the camera driver. No change on the robot is needed.

## 2. Daily use — the three commands

Inside the container (`docker compose exec bluerov2 bash`), after plugging in the battery:

```bash
# 1. START: waits until everything is reachable, then drivers, cameras, a light flash, checks
ros2 launch bluerov2_bringup session_start.launch.py

# 2. FOXGLOVE (second shell), if you want the GUI; it logs a lot, so it has its own terminal
ros2 launch bluerov2_bringup foxglove.launch.py

# 3. RECORD: Start/Stop in Foxglove (Record tab), or in a third shell (Ctrl-C to stop)
ros2 launch bluerov2_bringup record.launch.py bag_name:=dive01

# 4. STOP at the end: streams off, lights off, multi-camera computer powered off
ros2 launch bluerov2_bringup session_stop.launch.py
#    -> unplug the battery only after it prints "safe to unplug"
```

**`session_start.launch.py`** runs, in order, each step only if the previous one worked:

1. **preflight** — retries every 3 s (up to `wait_timeout`, 180 s) until the Pi answers
   ping, ArduSub sends its heartbeat, the DVL answers ping and TCP 16171, and the
   multi-camera computer answers ping and its camera software answers. A table shows
   which are ready.
2. **drivers** — `bluerov2.launch.py`.
3. **cameras** — starts the streams of the enabled multi-cameras (top pair by default).
4. **lights** — 50 % for 5 s, then off: the visual sign that the session started.
5. **stream check** — every started sensor is publishing at a plausible rate.
6. **SESSION READY**.

If a step fails it prints **SESSION NOT READY** and which step. Measured on the lab
vehicle: **ready 2 min 26 s after launch**, most of it waiting for the multi-camera
computer to boot. Keep this shell open: Ctrl-C here stops the drivers.

| `session_start` argument | Default | |
|---|---|---|
| `wait_timeout` | `180` | seconds preflight waits |
| `check_pi` / `check_imu` / `check_dvl` / `check_nvidia` | `true` | which preflight checks run |
| `camera` | `true` | nose camera driver + the QGC relay |
| `aux_left`, `aux_right` | `true` | multi-camera top pair: nodes and streams |
| `stereo_bottom`, `bottom_most` | `false` | multi-camera bottom pair |
| `foxglove` | `true` | Foxglove bridge |
| `lights_test` / `lights_test_power` / `lights_test_seconds` | `true` / `50` / `5` | the start flash |
| `lights` | `0` | lights level left on after the start |
| `check_timeout` | `15` | seconds the stream check waits per sensor |

**`session_stop.launch.py`** stops the streams that are running, sets the lights to 0 on
all channels, powers off the multi-camera computer over SSH and waits until it stops
answering (plus 10 s). It prints **SESSION STOPPED … safe to unplug** or **NOT SAFE TO
UNPLUG YET**. BlueOS is left on. Measured: 57 s.

### DVL calibration at the surface

Once the session is ready, float the vehicle **at the surface, disarmed and still**
(transducers in the water), then in a second shell:

```bash
ros2 run bluerov2_dvl dvl_calibrate --water-temp 12 --salinity 0   # fresh water, 12 °C
ros2 run bluerov2_dvl dvl_calibrate --water-temp 10 --salinity 35  # sea water
ros2 run bluerov2_dvl dvl_calibrate --dry-run                      # check only
```

Or from Foxglove, **DVL tab**: put `"12 0"` (temperature °C, salinity ppt) in *Set
water*, then **Calibrate DVL**. The button calls `/bluerov2/dvl/calibration/start`
(std_srvs/Trigger, node `dvl_calibration`, started by `bluerov2.launch.py`), which runs
the same `dvl_calibrate`; progress and result are on `dvl/calibration/status`.

It (1) reads the DVL config, (2) checks for 10 s that the vehicle is still — IMU
angular rate < 1 °/s and, if the DVL has bottom lock, DVL speed < 0.05 m/s — and aborts
if not, (3) sets the DVL sound speed from temperature and salinity (Medwin; a wrong sound
speed is a scale error on every velocity, ~3 % between fresh and sea water), (4) runs the
DVL's `calibrate_gyro` and checks the vehicle stayed still, (5) runs
`reset_dead_reckoning`, so `dvl/dead_reckoning` starts at 0 at the surface. A JSON record
goes to `~/dvl_calibration/`. Exit code 0 = OK, 1 = moved during calibration or error,
2 = not still (nothing changed). The script cannot see depth; being at the surface is up
to you. `dvl_node` can keep running: the script opens its own connection.

### Is the DVL data good?

```bash
ros2 run bluerov2_dvl dvl_health            # live verdict every 2 s
ros2 run bluerov2_dvl dvl_health --still    # vehicle still: also velocity noise and bias
```

It checks, over the last 5 s: report rate, bottom lock (`velocity_valid`), each of the 4
beams (`beam_valid`), `fom` (the DVL's own velocity standard deviation: < 0.01 m/s good,
< 0.05 usable), the altitude against the beam ranges it is computed from (mean range x
cos 22.5°), the spread of the 4 ranges (large on a flat floor = tilted vehicle or a beam
on a wall), speed > 1 m/s (AQUA-SLAM drops those) and, with `--still`, velocity noise
and bias. Each line is GOOD / CHECK / BAD with the reason. The checks are in
`bluerov2_dvl/health.py`.

## 3. First-time setup on a new laptop

1. **Clone and build** (build ~10 min; needs internet):
   ```bash
   git clone https://github.com/AdolfoCA/rov4_vslam.git && cd rov4_vslam
   docker compose build
   docker compose up -d
   ```
2. **Tether network**: the adapter facing the tether needs `192.168.2.1/24`,
   `192.168.1.1/24` and `10.42.0.1/24`, **no gateway** (TOPSIDE_SETUP.md, 2b):
   ```bash
   sudo nmcli connection modify "<tether profile>" ipv4.method manual \
     ipv4.addresses "192.168.2.1/24,192.168.1.1/24,10.42.0.1/24" ipv4.gateway ""
   sudo nmcli connection up "<tether profile>"
   ```
3. **Kernel UDP buffer** for raw video (TOPSIDE_SETUP.md, 2e):
   ```bash
   echo 'net.core.rmem_max=67108864' | sudo tee /etc/sysctl.d/90-ros-dds.conf && sudo sysctl --system
   ```
4. **Multi-camera login** for `session_stop` — copy the template and put the real
   password in the copy (it is in the lab's *Multi camera user guide*, "Getting the
   videos"). `data/` is git-ignored; **never put the password in the template or
   anywhere else in the repo — the repository is public**:
   ```bash
   cp ros2_ws/src/bluerov2_bringup/config/multicam_ssh.example data/.multicam_ssh
   chmod 600 data/.multicam_ssh
   nano data/.multicam_ssh            # password=<the real one>
   ```
   Without it, everything works except the power-off in `session_stop`, which says so.

After pulling new code, run `docker compose build` again whenever the `Dockerfile` or a
`package.xml` changed (new system packages), then `docker compose up -d`.

## 4. Running parts by hand

```bash
docker compose exec bluerov2 bash

# inside the container
ros2 launch bluerov2_bringup bluerov2.launch.py
```

This starts the IMU, DVL and nose camera drivers, the static TF node, the Foxglove bridge
and the **top pair** of the multi-camera system (Aux Left, Aux Right). It does not
start the multi-camera streams — use `multicam start` (section 5) or `session_start`.

Selective bring-up and overrides:

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py camera:=false     # no nose camera
ros2 launch bluerov2_bringup bluerov2.launch.py stereo_bottom:=true bottom_most:=true   # all 4 multicams
ros2 launch bluerov2_bringup bluerov2.launch.py aux_left:=false aux_right:=false        # no multicams
ros2 launch bluerov2_bringup bluerov2.launch.py dvl_ip:=192.168.2.95
```

After editing source on the host, rebuild inside the container with `build_ws` — the
workspace source is bind-mounted, so no image rebuild is needed.

## 5. Multi-camera system control — `multicam`

A command inside the container (an alias for `ros2 run bluerov2_bringup multicam.py`):

```bash
multicam status                     # cameras available / streaming / recording, lights state
multicam start                      # start the top pair (aux_left, aux_right)
multicam start all                  # all four;  or: multicam start stereo_bottom
multicam stop                       # stop the top pair;  multicam stop all = every streaming camera
multicam lights 40                  # lights to 40 % (0-100), channel 3
multicam lights 100 --channel 1     # channel 1-3
multicam lights 0                   # off
multicam shutdown                   # streams off, lights off, power off the computer
```

- **Streams must be started after every power-up of the camera computer**: they do not
  resume by themselves (checked). `session_start` does it for you.
- **Lights**: the light controller is a lifecycle node that boots `unconfigured`; in that
  state it accepts commands but the lights stay dark. `multicam lights` configures and
  activates it first when needed, then sets the power.
- It calls the camera system's `/ba/...` services with **CycloneDDS**, bound to the
  tether address `192.168.1.1`: the camera system does not answer service calls from
  Fast DDS, and replies sent to the laptop's Wi-Fi or Docker address never arrive. The
  drivers and the recorder stay on Fast DDS; topics work across both.
- `shutdown` reads the login from `data/.multicam_ssh` (section 3, step 4).

## 6. Recording

Record with `record.launch.py`, in a **second shell** inside the container while the
drivers run. Stop with **Ctrl-C**; the bag is closed cleanly.

```bash
docker compose exec bluerov2 bash

# 1. is everything I want to record actually publishing?
ros2 launch bluerov2_bringup record.launch.py check_only:=true

# 2. record (the same check runs first; recording starts only if it passes)
ros2 launch bluerov2_bringup record.launch.py bag_name:=dive01
```

**What is recorded is set in `ros2_ws/src/bluerov2_bringup/config/recording.yaml`.**
Topics are grouped by sensor; each group has `enabled: true|false`, its topics, and the
stream it is checked on. Edit and save; no rebuild needed. Defaults: IMU, DVL, dead
reckoning, the multi-camera top pair (JPEG), TF and logs on; nose camera and bottom pair
off. Any group can be switched for one recording from the command line:

```bash
ros2 launch bluerov2_bringup record.launch.py nose_camera:=true                     # + nose camera
ros2 launch bluerov2_bringup record.launch.py stereo_bottom:=true bottom_most:=true # + bottom pair
ros2 launch bluerov2_bringup record.launch.py dvl:=false                            # no DVL
```

A misspelled group name stops with the list of valid ones. Other arguments: `config`
(another YAML), `output_dir`, `bag_name`, `check`, `check_only`, `check_timeout`,
`storage`, `max_bag_duration`.

**How camera data is saved.** Images go into the same MCAP bag as every other sensor,
as ROS messages: per camera `camera_info` (calibration, one per frame) and
`image_raw/compressed` (each frame a JPEG, quality 90 for the multi-cameras, 85 for the
nose camera). Every frame keeps its `header.stamp`, on the same clock as IMU and DVL.
Raw `image_raw` is off by default (commented out in the YAML): it is lossless but
~39 MB/s per multi-camera and ~187 MB/s for the nose camera. To get image files out
of a bag, play it back and save the frames, or read the MCAP directly (e.g. with the
`mcap` / `rosbags` Python packages).

**From Foxglove.** `bluerov2.launch.py` also starts `recording_manager`, which runs this
same launch file for you: the **Record** tab of the layout has Start / Stop buttons, a
**Set folder** box (a subfolder of `data/`, e.g. `pool_test`), and the recorder's
status. The Overview tab has the Start / Stop bar too. Bags go to
`data/<folder>/rov_YYYYmmdd_HHMMSS`. Only `data/` is kept outside the container.

**To an external SSD.** The host's `/media` is mounted into the container at the same
path, so a plugged-in drive is at `/media/<user>/<drive>` inside too, even if it was
plugged in after `docker compose up`. Terminal: add
`output_dir:=/media/<user>/<drive>/rov`. Foxglove: type that absolute path into **Set
folder**. The welcome panel lists the drives it sees. The drive must be writable by
UID 1000: exFAT/NTFS drives mounted by the desktop are; an ext4 drive may need
`sudo chown -R $USER /media/$USER/<drive>` once on the host.

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

A multi-camera that should be recorded must also be **running and streaming**: turn its
node on in `bluerov2.launch.py` / `session_start.launch.py` too (`stereo_bottom:=true`
etc.), otherwise the check reports it missing.

**Disk.** Raw nose-camera video fills about 11 GB per minute; a 26 s test with every
sensor including raw nose video was 5 GB. The free space is printed when recording
starts.

## 7. Topics

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
| `dead_reckoning/odometry` | `nav_msgs/Odometry` | ~5 Hz | IMU heading + DVL velocity, `odom → base_link` (also on `/tf`) |
| `dead_reckoning/path` | `nav_msgs/Path` | 2 Hz, latched | that track |
| `dead_reckoning/dvl_path` | `nav_msgs/Path` | 2 Hz, latched | `dvl/dead_reckoning` placed in `odom`, for comparison |
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

## 8. Foxglove

Start the bridge in its own terminal (it is no longer part of the bring-up, because it
logs a lot): `ros2 launch bluerov2_bringup foxglove.launch.py`. Point Foxglove — the
desktop app, or the web app at app.foxglove.dev — at:

```
ws://<host running the container>:8765
```

The container uses host networking, so that is this machine's own address and no port
mapping is involved. Foxglove itself does not need ROS installed, and no DDS traffic
leaves the machine: the bridge multiplexes the entire graph over that one websocket.
This matters on a boat, where DDS multicast discovery across a tether or a shared lab
network is one of the more reliable ways to lose an afternoon.

**The layout is `ros2_ws/src/foxglove/rov_layout.json`** (Layout → Import from file), deliberately kept small:
Overview (cameras, recording bar, DVL health, velocity, altitude, attitude), Record
(folder, start/stop, status), DVL (health checklist and detail, velocity, altitude,
beam ranges), Cameras, 3D (dead-reckoning tracks, log). It is generated by `make_layout.js` in the same folder:
edit the panels there and run `node make_layout.js [namespace]` to regenerate it.

### Dead reckoning in the 3D panel

The **3D + log** tab of `rov_layout.json` is drawn in the `odom` frame and shows two
tracks, both from `dead_reckoning_node` (started by `bluerov2.launch.py`, turn off with
`dead_reckoning:=false`):

- **orange** `dead_reckoning/path`: DVL velocity, rotated by the vehicle IMU attitude
  (ArduSub, magnetometer heading) and the TF mounting, lever arm removed, integrated.
- **purple** `dead_reckoning/dvl_path`: the DVL's own dead reckoning (its own gyro),
  rotated and shifted onto the orange track at the first report and after each reset.

Both start where they are reset: `dvl_calibrate` resets both, or reset only the orange one
with `ros2 service call /bluerov2/dead_reckoning/reset std_srvs/srv/Trigger`. Without
bottom lock `dvl/velocity` stops and the position is held. The tracks should lie on top
of each other; if they fan apart, one heading is wrong (compass not calibrated, a DVL
yaw offset missing in `extrinsics.yaml`, magnetic disturbance near the pool). Both drift
and are for checking sensors, not for navigation.

### Video bandwidth

Raw `bgr8` at 1080p30 is about **186 MB/s**. No websocket will carry that, and Foxglove
will stall and then be dropped by the bridge's send buffer. The same frames as JPEG are
about **5 MB/s**, which is fine.

So the nose camera always publishes JPEG as well (`compressed_video:=true`, the
default; the multi-cameras always do), and you should point Foxglove's Image
panel at `camera/image_raw/compressed`, which the prepared layout already does. The
raw topic stays available for local consumers — a SLAM front end in the same container
pays no encode cost and reads `image_raw` directly.

`compressed_video:=false` saves the encode CPU when nobody watches or records JPEG.

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

## 8b. AQUA-SLAM

`ros2_ws/src/aqua_slam` is [AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM)
(Xu, Zhang, Wang, T-RO 2025), a tightly coupled stereo + DVL + gyro SLAM, ported from
ROS 1 to this container. What was changed and why: `ros2_ws/src/aqua_slam/UPSTREAM.md`.
It is GPL-3.0, unlike the rest of this repository.

```bash
# shell 1: drivers (or session_start.launch.py)
ros2 launch bluerov2_bringup bluerov2.launch.py
multicam start                    # the top pair must be streaming
# shell 2
ros2 launch aqua_slam aqua_slam.launch.py
```

It reads `multicam/aux_left` + `multicam/aux_right` (left/right image), `imu/data_raw`
and `dvl/report`, and publishes `/aqua_slam/orb_pose`, `orb_path`, `sparse_map`,
`img_with_info` (tracked features) and the TF `aqua_slam_map → aqua_slam_camera`.
On a bag: `use_sim_time:=true` and `ros2 bag play <bag> --clock`. The full argument list is
in the launch file's docstring.

**The camera intrinsics and the camera–IMU–DVL extrinsics in
`aqua_slam/config/bluerov2_multicam_top.yaml` are placeholders** (marked `[TODO]`):
the top pair is not calibrated yet and the multi-camera mounting is not measured.
The pipeline runs with them, but the trajectory means nothing until they are real.
Calibrate the pair in water, fill `LEFT.*`/`RIGHT.*` and `Camera.*`, and launch with
`rectify:=true`.

The vocabulary (`/opt/aqua_slam/Vocabulary/ORBvoc.txt`) is downloaded at image build.
AQUA-SLAM needs ~4 GB of RAM per compiler job, so the image and `build_ws` build with
1 job (~11 min for AQUA-SLAM); on a bigger machine, budget 4 GB per job:
`docker compose build --build-arg BUILD_JOBS=4`.

## 9. Frames

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

## 10. Timestamps

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
unsynchronised), so a stereo pair's frames have close but not identical times (12 ms
apart in one measured pair).

## 11. Before the first dive — the TODO list

The first group will stop the stack from working; the second will let it work while
producing quietly wrong numbers, which is worse.

**Will not work until set**

1. **`ROV_IP` / `DVL_IP`** — `docker-compose.yaml`. On the lab vehicle BlueOS is
   `192.168.2.2` and the DVL is set to a **fixed** `192.168.2.128` in its own web page
   (Configuration → Static; with DHCP it took up to 5 min after boot to get that
   address). Confirm with `ping`.
2. **MAVLink endpoint** — `connection_url` in `config/bluerov2.yaml`, default
   `udpin:0.0.0.0:14551`: BlueOS's "Multi-camera" endpoint sends an identical copy of
   the MAVLink stream there, leaving 14550 to QGroundControl. On a vehicle without that
   endpoint, use `udpin:0.0.0.0:14550` (and no QGC on the same laptop).
3. **Video port** — BlueOS sends the nose camera to `192.168.2.1:5600`; the relay in
   `bluerov2.launch.py` passes it to QGC (5600) and the driver (`udp_port` 5602).
4. **Topside network** — section 3: three fixed addresses on the tether adapter, no
   gateway, and `net.core.rmem_max`.
5. **Multi-camera login** — section 3, step 4 (`data/.multicam_ssh`), for
   `session_stop`.

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

## 12. Layout

```
rov4_vslam/
├── Dockerfile                     ROS 2 Humble + GStreamer + pymavlink + CycloneDDS + ssh
├── docker-compose.yaml            host networking, bind-mounted source, IPs
├── requirements.txt
├── rtps_udp_profile.xml           Fast DDS: plain UDPv4, 32 MB receive buffer
├── TOPSIDE_SETUP.md               what every topside laptop needs
├── data/                          recordings + .multicam_ssh (git-ignored)
└── ros2_ws/src/
    ├── bluerov2_msgs/             DVLReport, DVLBeam, DVLDeadReckoning
    ├── ba_msgs/                   multi-camera system services (Blue Atlas Robotics)
    ├── bluerov2_imu/              MAVLink driver + frames.py + time_sync.py
    ├── bluerov2_dvl/              TCP JSON driver, dvl_calibrate, dvl_health, dead reckoning
    ├── bluerov2_camera/           GStreamer driver (H.264 / H.265) + calibration.py
    ├── bluerov2_tf/               static TF node + extrinsics.py (parse & validate)
    ├── aqua_slam/                 stereo + DVL + gyro SLAM (section 8b)
    ├── foxglove/                  rov_layout.json (import in Foxglove) + make_layout.js
    └── bluerov2_bringup/          launch, scripts and configuration
        ├── launch/session_start.launch.py   preflight + drivers + cameras + checks
        ├── launch/session_stop.launch.py    streams off, lights off, camera computer off
        ├── launch/record.launch.py          stream check + rosbag recording
        ├── launch/bluerov2.launch.py        drivers + multicams + QGC relay + TF + DVL tools
                                             + recording_manager
        ├── launch/foxglove.launch.py        the bridge on its own
        ├── scripts/preflight.py             waits for Pi, IMU, DVL, camera computer
        ├── scripts/multicam.py              the `multicam` command
        ├── scripts/check_streams.py         the stream check
        ├── scripts/recording_manager.py     start/stop recordings from Foxglove
        ├── config/bluerov2.yaml             driver parameters (incl. the 4 multicams)
        ├── config/recording.yaml            which topics are recorded
        ├── config/multicam_ssh.example      template for data/.multicam_ssh
        ├── config/extrinsics.yaml           sensor mounting — measure these
        ├── config/foxglove_bridge.yaml      bridge parameters
        └── config/camera_info.yaml          placeholder calibration
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

## 13. Troubleshooting

**One sensor did not start?** In a container shell, `help_me` prints for each sensor
how to check it, what to do when its node runs but publishes nothing, and how to start
it on its own: `rov_start <sensor>` (`imu`, `dvl`, `camera`, `aux_left`, `aux_right`,
`stereo_bottom`, `bottom_most`, `tf`, `recording`) runs `bluerov2.launch.py` with only
that sensor on, and refuses if its node is already running.

| Symptom | Where to look |
|---|---|
| `PREFLIGHT FAILED … not ready: <system>` | that system did not answer within `wait_timeout`: power, cables, and the tether addresses (section 3); raise `wait_timeout` if it is just slow |
| Preflight waits for the DVL for minutes | the DVL is on DHCP again: set the fixed address in its web page (Configuration → Static IP) |
| `SESSION NOT READY: step '<name>' failed` | read the output of that step above the message; the drivers keep running |
| `No MAVLink heartbeat within 10 s` | `ping $ROV_IP`; `tcpdump -i any udp port 14551`; check the BlueOS endpoints (a udpout to `192.168.2.1:14551` must exist) |
| QGroundControl has no vehicle or no video | start QGC with the drivers running and check nothing else holds 14550 / 5600: `ss -lunp \| grep -E ':(14550\|5600) '` |
| IMU connects but topics are slow | ArduSub refused the interval request; watch the 5 s rate log |
| `No DVL reports in the last 5 s` | `nc -vz $DVL_IP 16171`; the DVL's own web UI |
| DVL velocity topic silent, `dvl/report` alive | no bottom lock — expected out of range or over soft mud |
| No frames (nose camera) | the relay needs the laptop to own `192.168.2.1`; `gst-launch-1.0 udpsrc port=5602 ! fakesink -v` inside the container |
| Video tears or stutters | raise `jitter_buffer_ms` |
| Raw frames at 1–3 fps in a bag or another node | `net.core.rmem_max` not raised on the host, see TOPSIDE_SETUP.md |
| `STREAM CHECK FAILED` before recording | the named sensor is not publishing: start it, or switch it off in `record.launch.py` |
| Multi-camera node logs `No frames` | streams not started (`multicam start`), or the host does not own `192.168.1.1`; `tcpdump -i any -c 5 udp portrange 5700-5703` |
| Multi-camera computer not answering | `ping -c3 10.42.0.5` must show `ttl=64`; replies with a lower ttl come from another machine over Wi-Fi |
| `multicam` says a service was not found / no reply | camera computer still booting; or the laptop lacks `192.168.1.1` / `10.42.0.1` |
| Lights do not turn on | `multicam status` shows the light controller state; `multicam lights` activates it |
| `session_stop` says `cannot read …/.multicam_ssh` or `template password` | section 3, step 4 |
| `session_stop`: `login or sudo refused` | wrong password in `data/.multicam_ssh` |
| TF lookups fail or give odd answers | `ros2 run tf2_tools view_frames`; check the `static_tf_node` startup log for warnings |
| Nodes cannot see each other | check `ROS_DOMAIN_ID` and that `FASTRTPS_DEFAULT_PROFILES_FILE` points at a valid file |
| Foxglove will not connect | `ss -ltnp \| grep 8765` in the container; check the host firewall; confirm the URL is `ws://` not `http://` |
| Foxglove connects, then drops during video | you are subscribed to raw `image_raw` — switch the Image panel to `image_raw/compressed` |
| Foxglove sees no topics | the bridge is up but the drivers are not; check `ros2 topic list` |
