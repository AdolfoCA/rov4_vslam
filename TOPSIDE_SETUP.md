# Topside setup — what every laptop needs

Notes from the hardware bring-up: first bring-up 2026-09-15, multi-camera system and
session launches added 2026-09-24. The day-to-day usage is in README.md (sections 2–6).

Vehicle: BlueROV2 Heavy + Blue Atlas multi-camera system. Topside: Ubuntu 20.04,
Docker 26.1.3.

---

## 1. Why any setup is needed at all

The sensors reach topside in different ways, and only one of them is
zero-configuration:

| Sensor | Direction | Needs topside config? |
|---|---|---|
| DVL-A50 | container dials **out**, TCP 16171 | no — works from any address |
| IMU (MAVLink) | BlueOS **pushes** to `192.168.2.1:14550` (QGC) and an identical copy to `192.168.2.1:14551` (IMU driver) | **yes** |
| Nose camera (H.264 RTP) | BlueOS **pushes** to `192.168.2.1:5600`; a relay on the laptop copies it to QGC (5600) and the driver (5602) | **yes** |
| Multi-camera system (H.265 RTP, 4 cameras) | the camera computer **pushes** to `192.168.1.1:5700-5703`, once started with `multicam start` | **yes** |

Because BlueOS pushes MAVLink and video at the fixed address `192.168.2.1`, the topside
machine must actually hold that address or those two sensors receive nothing. Confirmed
from BlueOS itself:

```
$ curl -s http://192.168.2.2/ardupilot-manager/v1.0/endpoints/
GCS Client Link   udpout  192.168.2.1:14550   enabled=True
Multi-camera      udpout  192.168.2.1:14551   enabled=True

$ curl -s http://192.168.2.2/mavlink-camera-manager/streams
"endpoints": ["udp://192.168.2.1:5600"], H264 1920x1080@30, running
```

Switching the IMU to dial out instead (`ros2 launch ... rov_ip:=192.168.2.2`, which
selects `udpout:192.168.2.2:14550`) was tested and does **not** get a heartbeat, so the
static address is currently required.

The same applies to the multi-camera system: its streams go to `192.168.1.1`, and its
computer (`10.42.0.5` / `192.168.1.4`) uses `10.42.0.1` as its gateway. See 2b.

**QGroundControl on the same laptop.** A UDP port feeds only one program, and QGC wants
14550 and 5600. So the IMU driver listens on 14551 (the "Multi-camera" endpoint above
already sends an identical copy: 1107 vs 1111 packets in 5 s), and the nose-camera
driver gets its video from a relay started by `bluerov2.launch.py`: it binds
`192.168.2.1:5600` (Linux prefers that over QGC's `0.0.0.0:5600`) and copies each
packet, undecoded, to `127.0.0.1:5600` (QGC) and `127.0.0.1:5602` (driver). Tested:
QGC connected and showed video while the IMU driver ran at 101 Hz and the nose-camera
driver at 32 fps. No change on the robot or in QGC's settings.

---

## 2. Per-laptop setup — run once

**a. Ethernet.** A USB→Ethernet adapter is fine; the tether interface board is RJ45.
Find the interface name:

```bash
ip -br addr        # the tether NIC shows up as enxXXXXXXXXXXXX
```

The Fathom-X topside box is **powered by its USB cable**: keep the USB cable plugged
into the laptop even when the network goes through its Ethernet port. Unplugging the
USB drops the whole tether link.

**b. Give the tether adapter its three fixed addresses, permanently, and no gateway.**

| Address | Why |
|---|---|
| `192.168.2.1/24` | BlueOS sends MAVLink (IMU) and the nose-camera video here |
| `192.168.1.1/24` | the multi-camera system sends its 4 video streams here |
| `10.42.0.1/24` | the multi-camera computer's network; it uses `10.42.0.1` as its gateway, and replies from it get stuck if nobody owns that address |

```bash
nmcli -t -f NAME,DEVICE con show --active | grep <your-enx-interface>
sudo nmcli connection modify "Wired connection 3" \
  ipv4.method manual \
  ipv4.addresses "192.168.2.1/24,192.168.1.1/24,10.42.0.1/24" \
  ipv4.gateway ""
sudo nmcli connection up "Wired connection 3"
```

Substitute the profile name from the first command. Use `nmcli`, not
`sudo ip addr add ... dev <iface>` — the `ip` form is lost on the next reboot, replug or
DHCP renew. This was observed mid-session: the address silently disappeared and both
IMU and camera went dead while the DVL kept working. `nmcli connection modify` only
saves the profile; it takes effect after `nmcli connection up` (or a replug).

Verify:

```bash
$ ip -br addr show <iface>
enx606d3cf5a945  UP  192.168.2.1/24 192.168.1.1/24 10.42.0.1/24 ...
$ ip route | grep default          # must still be via Wi-Fi, NOT via the tether
```

Pitfalls met on 2026-09-22/24:
- **No gateway on the tether profile.** A gateway there becomes the preferred route
  (metric 100 vs 600 for Wi-Fi) and sends all internet traffic into the ROV.
- **`/24`, never `/8`.** `10.42.0.x/8` claims all of `10.0.0.0/8`, which includes the
  university Wi-Fi network (`10.59.x`, `10.208.x`).
- **Do not copy another laptop's addresses.** `10.42.0.156` was a colleague's *laptop*
  address, not the camera's; pinging it from his laptop was pinging himself.
- **One laptop at a time** on the tether with `192.168.1.1` / `10.42.0.1`.
- **`10.42.x.x` also exists on the university network.** Without the tether addresses,
  a ping to `10.42.0.5` goes out over Wi-Fi and gets answers from other machines
  (`ttl=55`). A device really on the tether answers with **`ttl=64`**.

**c. Docker Compose v2.** Ubuntu's `docker.io` package does not ship it. No sudo needed:

```bash
mkdir -p ~/.docker/cli-plugins
curl -sSL -o ~/.docker/cli-plugins/docker-compose \
  https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64
chmod +x ~/.docker/cli-plugins/docker-compose
docker compose version
```

**d. Disk.** The image is ~3 GB built; allow ~8 GB free for a build. Every rebuild leaves
the previous image behind: reclaim it with `docker image prune` (removes only unused
images).

**e. Allow large UDP receive buffers, permanently.** Without this, raw camera frames
(`camera/image_raw`, 6.2 MB each) are lost by any process other than the camera node
itself: `ros2 bag record`, `ros2 topic hz`, a VSLAM node. Requires sudo, once per laptop:

```bash
echo 'net.core.rmem_max=67108864' | sudo tee /etc/sysctl.d/90-ros-dds.conf
sudo sysctl --system
sysctl net.core.rmem_max        # must print 67108864
```

`rtps_udp_profile.xml` asks Fast DDS for a 32 MB receive buffer, but the kernel silently
caps that request at `rmem_max`, whose default is ~208 KB. Both halves are needed: the
limit alone changes nothing (measured: 11 of 184 frames), the profile alone is capped.
See section 7, fix 4, for the measurements.

**f. Multi-camera computer login, for `session_stop`.** `multicam shutdown` powers the
camera computer off over SSH (`sshpass` + `ssh nvidia@10.42.0.5 sudo poweroff`) and reads
the login from `rov4_vslam/data/.multicam_ssh`. Create it from the template and put the
real password in the copy — it is in the lab's *Multi camera user guide*, section
"Getting the videos":

```bash
cd rov4_vslam
cp ros2_ws/src/bluerov2_bringup/config/multicam_ssh.example data/.multicam_ssh
chmod 600 data/.multicam_ssh
nano data/.multicam_ssh            # host=10.42.0.5  user=nvidia  password=<real one>
```

`data/` is git-ignored and bind-mounted into the container (`/home/rosdev/data`), so the
file is visible to `multicam` but never committed. **Never put the real password in the
template or anywhere else in the repository — it is public.** If the file is missing or
still has `CHANGE_ME`, `session_stop` stops before the power-off and says so.

---

## 3. Per-vehicle configuration

**DVL address — fixed, set in the DVL itself.** The DVL was on DHCP: it only got
`192.168.2.128` once BlueOS handed it out, which on 2026-09-24 took 5 minutes after the
Pi was up (until then it only had its factory address `192.168.194.95`, so preflight
waited 441 s). It is now set **static** in its web page:
http://192.168.2.128 → **Configuration** → *Static or dynamic IP* → **Static**,
address `192.168.2.128`, prefix 24, gateway/DNS `192.168.2.2` → **Apply**, then a power
cycle. It is ready at power-on since. The factory address `192.168.194.95` always stays
as a backup: to reach it, give the laptop a temporary `192.168.194.1/24`. On another
vehicle, find the DVL with:

```bash
avahi-resolve -4 -n waterlinked-dvl.local            # the DVL announces itself
for i in $(seq 1 254); do (ping -c1 -W1 192.168.2.$i >/dev/null 2>&1 && echo "192.168.2.$i UP") & done; wait
curl -s http://<candidate>/api/v1/ip                  # saved network settings
```

**Multi-camera computer — already fixed.** Hostname `adequate-mantine`, MAC
`48:b0:2d:3a:88:8d`. Its NetworkManager profile `nm_conn_static` (the only one that
autoconnects) sets `192.168.1.4/24` and `10.42.0.5/24`, gateway `10.42.0.1`. Three other
profiles exist but are off (`custom_static`: `192.168.2.5/24` + `10.42.0.5/8`; two DHCP
profiles) — do not activate them, the addresses would change. Streams: `aux_left` 5700,
`aux_right` 5701, `stereo_bottom` 5702, `bottom_most` 5703 — in `config/bluerov2.yaml`.

---

## 4. Bring-up

```bash
cd rov4_vslam
docker compose build          # first time, and after Dockerfile / package.xml changes
docker compose up -d
docker compose exec bluerov2 bash
# inside:
ros2 launch bluerov2_bringup session_start.launch.py     # after plugging in the battery
```

`session_start` waits until Pi, IMU heartbeat, DVL and camera computer are reachable,
then starts the drivers, the camera streams, flashes the lights and checks every stream
(README.md, section 2). Measured 2026-09-24 with the fixed DVL address: Pi and IMU after
69 s, camera computer after 128 s, **SESSION READY after 146 s**.

To run only the drivers: `ros2 launch bluerov2_bringup bluerov2.launch.py` (IMU, DVL,
nose camera + relay, top pair of the multi-camera system; the streams are then started
with `multicam start`).

**Only one process may bind UDP 5600 (on 192.168.2.1), 5602, 5700-5703 and 14551.** If
you ran `gst-launch` or a second launch to test the video, stop it first or the node gets
nothing and looks broken. Stale nodes from an earlier launch cause the same symptom (on
2026-09-24 a killed launch left its nodes running: the IMU showed 202 Hz, two nodes);
`docker compose restart bluerov2` clears them.

At the end: `ros2 launch bluerov2_bringup session_stop.launch.py`, then unplug when it
prints "safe to unplug" (measured: 57 s). BlueOS stays on.

---

## 5. Checking it works, before trusting it

```bash
# raw bytes arrive at the laptop, before ROS is involved
sudo timeout 5 tcpdump -i any -n udp port 14551            # MAVLink (IMU driver's copy)
sudo timeout 5 tcpdump -i any -n udp port 5600             # nose camera video
sudo timeout 5 tcpdump -i any -n udp portrange 5700-5703   # multi-camera video
timeout 5 nc 192.168.2.128 16171                           # DVL JSON
ping -c3 10.42.0.5                                         # camera computer, ttl=64

# then, inside the container
ros2 run bluerov2_bringup preflight.py --timeout 30 pi imu dvl nvidia   # before the drivers
multicam status                                                         # camera computer
ros2 launch bluerov2_bringup record.launch.py check_only:=true          # all sensor rates
```

Stock tooling for single topics:

```bash
ros2 topic list
ros2 topic hz   /bluerov2/imu/data_raw
ros2 topic echo /bluerov2/imu/data_raw --field linear_acceleration --once
ros2 run rqt_image_view rqt_image_view      # X11 is already wired up in compose
```

---

## 6. Recording and the multi-camera system

### 6.1 Recording

`bluerov2_bringup/launch/record.launch.py` records the selected sensors into a rosbag
(MCAP), after checking that each of them is publishing. Run it in a second shell while
the drivers are up; stop it with Ctrl-C, which closes the bag cleanly.

```bash
ros2 launch bluerov2_bringup record.launch.py check_only:=true     # is everything on?
ros2 launch bluerov2_bringup record.launch.py bag_name:=dive01     # check, then record
ros2 launch bluerov2_bringup record.launch.py camera:=true         # + nose camera
ros2 launch bluerov2_bringup record.launch.py stereo_bottom:=true bottom_most:=true
```

**Defaults: IMU, DVL, `aux_left`, `aux_right` on; nose camera, `stereo_bottom`,
`bottom_most` off.** Multi-camera frames are recorded as JPEG (`multicam_compressed`);
add `multicam_raw:=true` for uncompressed 960×540 frames. The full argument table is in
README.md, section 6.

The stream check waits up to `check_timeout` (10 s) for each selected sensor and
measures its rate (IMU ≥ 50 Hz, DVL ≥ 1 Hz, cameras ≥ 10 Hz). If one is missing or too
slow, it prints `STREAM CHECK FAILED: <sensor>` and **no recording is started**.

- All sensors go into one bag on the same ROS clock. Nothing is resampled or dropped to
  align them: every message keeps its own rate, its driver's `header.stamp`, and the
  time it was recorded. Align sensors offline by `header.stamp`.
- A topic nobody publishes is simply absent from the bag. In air `dvl/velocity` and
  `dvl/altitude` record 0 messages; that is normal. Nothing publishes `/tf` yet.
- **Disk.** Raw nose-camera video is ~187 MB/s, about 11 GB per minute; JPEG is ~15 MB/s.
  The free space is printed when recording starts.
- Raw frames need step 2e above, or `image_raw` records at ~1–2 fps.
- It uses rosbag2's C++ recorder rather than a Python node, which could not keep up
  with raw video.

### 6.2 Multi-camera system

- A separate NVIDIA computer on the ROV network, with its own battery, running ROS 2
  nodes under `/ba/...` on **CycloneDDS** (`camera_feed_manager`, `light_controller`,
  `leak_detector`, `battery_monitor`, `log_dir_manager`). Its message definitions are
  in `ros2_ws/src/ba_msgs`.
- It sends one H.265 stream per camera to `192.168.1.1`, **960×540 at 30 fps** (the
  stream header says 25). The drivers here only receive them: one `camera_node` per
  camera with `codec: h265`, topics `/bluerov2/multicam/<camera>/...`.
- **The streams do not survive a reboot of the camera computer** (checked: 4 min after a
  reboot its software was up but nothing streamed). They are started with
  `/ba/camera_feed_manager/new_attach_streaming_consumer`, one call per camera — done by
  `multicam start` / `session_start`.
- **Its services only answer CycloneDDS.** From Fast DDS the requests arrive but no reply
  ever comes back (topics do work across both). `multicam` therefore re-executes itself
  with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and CycloneDDS bound to `192.168.1.1`;
  everything else stays on Fast DDS.
- **Lights**: `/ba/light_controller` is a lifecycle node that boots `unconfigured`; in that
  state `set_brightness` answers `done=True` but the lights stay dark. `multicam lights`
  runs configure → activate first when needed. Checked with the lights at 50 %.
- The camera system's own recording (`start_recording`) is synchronised and stays on
  the NVIDIA (`/storage/logs/<date>/mission_*`); the streams we record are not
  synchronised with each other.

---

## 7. Code fixes this bring-up required

Fixes 1–3 were blockers found during bring-up; none of the code had ever run against
hardware. Fix 4 was found later the same day, while adding recording.

1. **`Dockerfile`** — `COPY --chown` sets ownership on `ros2_ws/src`, but Docker creates
   the parent `ros2_ws` as `root:root`, so the unprivileged user cannot create
   `build/ install/ log/`. `colcon build` died with
   `PermissionError: [Errno 13] Permission denied: 'log'`. Fixed with
   `RUN chown $UID:$GID /home/$USERNAME/ros2_ws` before the `USER` switch.

2. **`bluerov2_imu/imu_node.py`** — `self._clock = ClockOffsetEstimator(...)` shadows
   `rclpy.node.Node._clock`, so `create_timer()` raised
   `AttributeError: 'ClockOffsetEstimator' object has no attribute 'handle'` and the
   node never started. Renamed to `self._clock_sync`. The unit tests miss this because
   they exercise `ClockOffsetEstimator` without a ROS node.

3. **`bluerov2_camera/camera_node.py`** — two separate faults:
   - Importing `gi`/GStreamer **after** `rclpy` makes the next `Node()` construction
     segfault inside the `_rclpy` C extension, with no Python traceback. The GStreamer
     import block must come first.
   - `image.data = frame.tobytes()` misses rosidl's only fast path
     (`array.array('B')`) and falls into a per-element validation loop over all 6.2 M
     bytes: **322 ms per frame, a 3 fps ceiling**. Changed to
     `array.array('B', frame.tobytes())` — byte-identical, ~95x faster.
     Measured effect: **1.0 fps -> 32 fps**.

4. **`rtps_udp_profile.xml`** — no socket buffer size was set, so every Fast DDS UDP
   socket got the kernel default of ~208 KB. A raw 1080p frame (6.2 MB) goes out as a
   burst of ~100 RTPS fragments; the receiving socket overflowed, and because the camera
   topic is `BEST_EFFORT`, a frame missing one fragment is dropped whole. Every process
   other than the camera node (Python subscribers and the C++ `ros2 bag record` alike)
   saw `image_raw` at 1–3 fps, while the small `camera_info` from the same callback
   arrived at 32 Hz. Fixed with `<receiveBufferSize>33554432</receiveBufferSize>` in the
   profile, **together with** raising `net.core.rmem_max` on the host (step 2e) — the
   kernel silently caps the request at that limit. Measured, 6 s of `ros2 bag record`
   on `image_raw` + `camera_info`, with the `RcvbufErrors` counter from `/proc/net/snmp`:

   | Setup | `image_raw` frames | UDP RcvbufErrors |
   |---|---|---|
   | before | 5 (vs 184 `camera_info`) | 2,156 |
   | `rmem_max` raised only | 11 (vs 184) | 2,116 |
   | `rmem_max` + profile, image rebuilt | **187 of 187** | **0** |

   `record.launch.py` with all defaults for 9.7 s: `image_raw`, `image_raw/compressed`
   and `camera_info` all 310 messages, 0 RcvbufErrors. To confirm the profile is live,
   the recorder's sockets show `rb67108864` in `ss -uamp` (Linux doubles the request).
   This also explains why enabling shared memory earlier changed nothing: Fast DDS only
   uses it when publisher *and* subscriber both enable it, and the camera node was
   still on the UDP-only profile.

Additions on 2026-09-24, not fixes: `camera_node` `codec` parameter (`h264` / `h265`);
multi-camera nodes and the nose-video relay in `bluerov2.launch.py`; per-sensor switches
and the stream check in `record.launch.py`; `session_start` / `session_stop` launches with
`preflight.py` and `multicam.py`; `ba_msgs` in the workspace (pulls in `mavros_msgs`);
CycloneDDS, `openssh-client` and `sshpass` in the image.

---

## 8. Open items

- **Multi-camera TF:** the four camera frames are not in the TF tree; measure their
  mounting.
- `camera_info.yaml` is a placeholder; `k[0] = 0`. Needs calibration **in water**, for
  the nose camera and for the multi-camera pair.
- `extrinsics.yaml` values are placeholders; `static_tf_node` warns about them at
  startup. Measure before any navigation work.
- Accelerometer magnitude reads |a| = 10.26 m/s^2 at rest, 4.5% above 9.81. Scaling
  constants in the driver are correct per the MAVLink spec, so this looks like an
  uncalibrated accelerometer — run the ArduSub accel calibration.
- Magnetometer reads |B| = 21 uT; expect roughly 47 uT in Italy. Suggests the
  magnetometer needs calibrating in ArduSub.
- IMU reports 59.5 C. Plausible for electronics in a sealed enclosure, but worth a look.
- DVL dead reckoning reports positions of hundreds of km with std of ~8e6 m. Expected
  with no bottom lock ever acquired, but must be re-checked in water before any SLAM
  work consumes that topic.
