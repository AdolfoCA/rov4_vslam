# Topside setup — what every laptop needs

Notes from the first hardware bring-up, 2026-09-15. To be folded into README.md later.

Vehicle: BlueROV2 Heavy. Topside: Ubuntu 20.04, Docker 26.1.3.

---

## 1. Why any setup is needed at all

The three sensors reach topside in three different ways, and only one of them is
zero-configuration:

| Sensor | Direction | Needs topside config? |
|---|---|---|
| DVL-A50 | container dials **out**, TCP 16171 | no — works from any address |
| IMU (MAVLink) | BlueOS **pushes** to `192.168.2.1:14550` | **yes** |
| Camera (H.264 RTP) | BlueOS **pushes** to `192.168.2.1:5600` | **yes** |

Because BlueOS pushes MAVLink and video at the fixed address `192.168.2.1`, the topside
machine must actually hold that address or those two sensors receive nothing. Confirmed
from BlueOS itself:

```
$ curl -s http://192.168.2.2/ardupilot-manager/v1.0/endpoints/
GCS Client Link   udpout  192.168.2.1:14550   enabled=True

$ curl -s http://192.168.2.2/mavlink-camera-manager/streams
"endpoints": ["udp://192.168.2.1:5600"], H264 1920x1080@30, running
```

Switching the IMU to dial out instead (`ros2 launch ... rov_ip:=192.168.2.2`, which
selects `udpout:192.168.2.2:14550`) was tested and does **not** get a heartbeat, so the
static address is currently required.

---

## 2. Per-laptop setup — run once

**a. Ethernet.** A USB→Ethernet adapter is fine; the tether interface board is RJ45.
Find the interface name:

```bash
ip -br addr        # the tether NIC shows up as enxXXXXXXXXXXXX
```

**b. Give it 192.168.2.1, permanently.** DHCP from BlueOS assigns something in the
`.100+` range, which is *not* where BlueOS pushes. Add `.1` alongside it.

```bash
nmcli -t -f NAME,DEVICE con show --active | grep <your-enx-interface>
sudo nmcli con mod "Wired connection 3" +ipv4.addresses 192.168.2.1/24
sudo nmcli con up  "Wired connection 3"
```

Substitute the profile name from the first command. Use `nmcli`, not
`sudo ip addr add 192.168.2.1/24 dev <iface>` — the `ip` form is lost on the next
reboot, replug or DHCP renew. This was observed mid-session: the address silently
disappeared and both IMU and camera went dead while the DVL kept working.

Verify — both addresses should be listed:

```bash
$ ip -br addr show <iface>
enx606d3cf5a945  UP  192.168.2.135/24 192.168.2.1/24 ...
```

**c. Docker Compose v2.** Ubuntu's `docker.io` package does not ship it. No sudo needed:

```bash
mkdir -p ~/.docker/cli-plugins
curl -sSL -o ~/.docker/cli-plugins/docker-compose \
  https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64
chmod +x ~/.docker/cli-plugins/docker-compose
docker compose version
```

**d. Disk.** The image is ~2.9 GB built; allow ~8 GB free for the build.

---

## 3. Per-vehicle configuration

**DVL address.** The factory default `192.168.2.95` was **not** correct on this
vehicle — it answers at **192.168.2.128**. Already changed in `docker-compose.yaml`
and `config/bluerov2.yaml`. On a different vehicle, find it with:

```bash
for i in $(seq 1 254); do (ping -c1 -W1 192.168.2.$i >/dev/null 2>&1 && echo "192.168.2.$i UP") & done; wait
curl -s http://<candidate>/        # the DVL answers with "Water Linked DVL GUI"
```

---

## 4. Bring-up

```bash
cd rov4_vslam
docker compose build          # first time only
docker compose up -d
docker compose exec bluerov2 bash
# inside:
ros2 launch bluerov2_bringup bluerov2.launch.py
```

**Only one process may bind UDP 5600 and 14550.** If you ran `gst-launch` or a second
launch to test the video, stop it first or the node gets nothing and looks broken.
Stale nodes from an earlier launch cause the same symptom; `docker compose restart
bluerov2` clears them.

---

## 5. Checking it works, before trusting it

```bash
# raw bytes arrive at the laptop, before ROS is involved
sudo timeout 5 tcpdump -i any -n udp port 14550     # MAVLink
sudo timeout 5 tcpdump -i any -n udp port 5600      # video
timeout 5 nc 192.168.2.128 16171                    # DVL JSON

# then, inside the container
python3 ~/data/verify_all.py      # one row per topic: message count + a sample value
python3 ~/data/grab_ros_frame.py  # saves a frame from /camera/image_raw to data/
```

> The two scripts above are local scratch helpers written during the bring-up. They
> live in `data/`, which this repo gitignores, so they are **not** part of the
> checkout — they are mentioned only to record how the numbers below were obtained.
> The equivalent with stock tooling:
>
> ```bash
> ros2 topic list
> ros2 topic hz   /bluerov2/imu/data_raw
> ros2 topic echo /bluerov2/imu/data_raw --field linear_acceleration --once
> ros2 run rqt_image_view rqt_image_view      # X11 is already wired up in compose
> ```

---

## 6. Code fixes this bring-up required

All three were blockers; none had ever run against hardware.

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

---

## 7. Open items

- `camera_info.yaml` is a placeholder; `k[0] = 0`. Needs calibration **in water**.
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
- A Python subscriber reads `/camera/image_raw` at only ~3 fps while the node publishes
  at 32 fps (its own counter, and `camera_info` from the same callback measures 32 Hz).
  Not diagnosed. Enabling shared memory did not change it. Matters only if a consumer
  needs raw 1080p frames from a *separate* process.
