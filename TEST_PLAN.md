# BlueROV2 Heavy — sensor bring-up test plan

**Goal:** confirm the vehicle is reachable, that all three sensors deliver data into ROS,
and that the data is *correct* — not merely present.

**Status of the code you are testing:** it has never run against hardware. The 60 unit
tests cover the maths and the wire formats; nothing has touched a real vehicle, and the
Docker image has not been built. Expect to find problems. That is the point of today.

**Time:** about 90 minutes dry, plus 30 in water.

---

## How to use this document

Work through the stages **in order** and do not skip ahead. Each stage assumes the one
before it passed. This matters more than it sounds: if you launch everything at once and
see no IMU data, the cause could be the network, BlueOS, the container, the driver, or a
frame convention — five candidates. Verified layer by layer, there is only ever one.

> **STOP rule.** If a stage fails, stop and fix it. Do not continue with "we'll come back
> to that". A later stage built on a broken earlier one produces misleading symptoms.

Boxes marked **[R]** need a value written into the results sheet at the end.

---

## Stage 0 — Before you leave

Kit:

- [ ] Topside laptop with Docker installed and **the image already built** — `docker compose build` pulls several hundred MB and you do not want to discover the dockside wifi is bad. Build it at home.
- [ ] Ethernet cable, tether interface board, PSU
- [ ] Tape measure (for the DVL altitude check)
- [ ] A phone with a compass app (for the heading check)
- [ ] Something to write on

Build and smoke-test at home, on the bench, before travelling:

```bash
cd rov4_vslam
docker compose build
docker compose up -d
docker compose exec bluerov2 bash
# inside:
colcon test --packages-select bluerov2_imu bluerov2_dvl bluerov2_tf
colcon test-result --verbose
```

- [ ] **[R]** Image builds without error
- [ ] **[R]** 60 tests pass

If the build fails, that is a code problem, not a vehicle problem — send the error and it
can be fixed before you go.

---

## Stage 1 — Network

Power the vehicle, connect the tether, and give the topside machine a static address on
the vehicle's subnet.

```bash
# Topside interface facing the tether board. Replace eth0 with yours (check: ip addr).
sudo ip addr add 192.168.2.1/24 dev eth0
sudo ip link set eth0 up
```

| Check | Command | Pass |
|---|---|---|
| ROV reachable | `ping -c 4 192.168.2.2` | 4 replies, < 10 ms |
| DVL reachable | `ping -c 4 192.168.2.95` | 4 replies |
| BlueOS web UI | open `http://192.168.2.2` in a browser | page loads |

- [ ] **[R]** ROV ping: ___ ms average, ___ % loss
- [ ] **[R]** DVL ping: ___ ms average, ___ % loss

**If the DVL does not answer**, its address may differ from the factory default. Find it
in BlueOS (the DVL extension page) or on the Water Linked web UI, and note the real
address — you will need it in Stage 4.

- [ ] **[R]** Actual DVL address: ______________

While you have BlueOS open, confirm in its interface:

- [ ] Autopilot is connected and ArduSub is running
- [ ] A video stream exists, and **note which UDP port it targets** (default 5600)
- [ ] **[R]** Video port: ______  Resolution: __________

---

## Stage 2 — Raw data, before ROS is involved

This is the stage people skip, and it is the one that saves the afternoon. Prove the bytes
arrive at the laptop *before* asking whether ROS sees them. Run these **on the topside
host**, outside the container.

**MAVLink:**
```bash
sudo timeout 5 tcpdump -i any -n udp port 14550
```
- [ ] **[R]** Packets arriving: yes / no, roughly ___ per second

**Video:**
```bash
sudo timeout 5 tcpdump -i any -n udp port 5600
```
- [ ] **[R]** Packets arriving: yes / no

**DVL:**
```bash
timeout 5 nc 192.168.2.95 16171
```
You should see a wall of JSON lines. Look at one and confirm it has a `"type"` field.
- [ ] **[R]** JSON arriving: yes / no
- [ ] **[R]** `"format"` value seen: ______________

> **If MAVLink shows nothing:** BlueOS decides where to send it. In BlueOS go to
> Autopilot → Endpoints and confirm there is a UDP endpoint aimed at `192.168.2.1:14550`.
> If instead BlueOS is set up to *wait* for a client, you will need `rov_ip:=192.168.2.2`
> at launch in Stage 4, which switches the driver to connecting outward.

> **If video shows nothing:** the stream is not started, or targets a different host or
> port. Fix it in the BlueOS video page before continuing.

---

## Stage 3 — Container up

```bash
cd rov4_vslam
docker compose up -d
docker compose exec bluerov2 bash
# inside the container:
build_ws
```

- [ ] **[R]** `colcon build` finishes clean
- [ ] `ros2 pkg list | grep bluerov2` shows six packages

If two laptops are on the same network running ROS, set a distinct `ROS_DOMAIN_ID` in
`docker-compose.yaml` on each, or they will see each other's topics and confuse everyone.

---

## Stage 4 — Sensors, one at a time

Bring each driver up **alone** first. If everything is launched together, four sets of log
messages interleave and a failure in one looks like a failure in another.

### 4a. IMU

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py dvl:=false camera:=false foxglove:=false
```

Watch the startup log. Within ten seconds you want:

```
Heartbeat from system 1, component 1 (autopilot type ...)
```

then, every five seconds, a rate line like `ATTITUDE_QUATERNION 50 Hz, RAW_IMU 100 Hz, clock offset +1234.567 s`.

In a second shell (`docker compose exec bluerov2 bash`):

```bash
ros2 topic hz /bluerov2/imu/data_raw --window 100
ros2 topic hz /bluerov2/imu/data --window 50
ros2 topic echo /bluerov2/imu/data_raw --once
```

**Vehicle level, still, on the bench, thrusters off:**

| Quantity | Expected | Pass band | Result |
|---|---|---|---|
| `imu/data_raw` rate | 100 Hz | ≥ 40 Hz and steady | **[R]** ___ |
| `imu/data` rate | 50 Hz | ≥ 20 Hz and steady | **[R]** ___ |
| `linear_acceleration` | (0, 0, **+9.81**) | \|a\| = 9.81 ± 0.3 m/s² | **[R]** ___ |
| `angular_velocity` | (0, 0, 0) | each \|ω\| < 0.03 rad/s | **[R]** ___ |
| `imu/mag` magnitude | ~50 µT in Denmark | 20–80 µT | **[R]** ___ |
| `imu/pressure` | ~101325 Pa | 95 000–105 000 Pa | **[R]** ___ |
| `clock offset` | any value | drifts < 10 ms over a minute | **[R]** ___ |

> **The sign of z matters more than anything else here.** An accelerometer at rest measures
> *specific force*, which points **up**, so a level vehicle must read **+9.81** on z. If you
> see **−9.81**, the FRD→FLU conversion is inverted somewhere and every downstream estimate
> will be wrong. Stop and report it.

> **Rates below 40 Hz** usually mean ArduSub declined the interval request rather than a
> link problem. Note the number and carry on — it is a tuning issue, not a blocker.

- [ ] **[R]** IMU stage: pass / fail — notes: ______________

### 4b. DVL

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py imu:=false camera:=false foxglove:=false \
  dvl_ip:=192.168.2.95
```

> **Read this before you panic.** A DVL out of water **cannot get bottom lock**. On the
> bench you should expect `velocity_valid: false`, invalid beams, and *no messages at all*
> on `/bluerov2/dvl/velocity` — the driver drops invalid reports on purpose, because a gap
> is safer for an estimator than a confidently wrong velocity. **This is correct behaviour,
> not a fault.** The test on the bench is only that reports arrive at all.

**Dry (bench):**

```bash
ros2 topic hz /bluerov2/dvl/report
ros2 topic echo /bluerov2/dvl/report --once
```

| Check | Expected | Result |
|---|---|---|
| `dvl/report` arriving | yes, a few Hz | **[R]** ___ |
| `velocity_valid` | `false` | **[R]** ___ |
| `beams` array length | 4 | **[R]** ___ |
| `/bluerov2/dvl/velocity` | silent | **[R]** ___ |

**In water, vehicle stationary, ≥ 0.5 m above the bottom:**

```bash
ros2 topic echo /bluerov2/dvl/velocity --once
ros2 topic echo /bluerov2/dvl/altitude --once
```

| Check | Expected | Pass band | Result |
|---|---|---|---|
| `velocity_valid` | `true` | — | **[R]** ___ |
| beams valid | 4 of 4 | ≥ 3 | **[R]** ___ |
| `altitude` | matches tape measure | ± 0.15 m | **[R]** ___ |
| velocity while stationary | (0, 0, 0) | each \|v\| < 0.05 m/s | **[R]** ___ |
| `fom` | small | < 0.05 m/s | **[R]** ___ |
| `dvl/velocity` rate | ≤ 15 Hz | ≥ 2 Hz | **[R]** ___ |

The altitude check against a tape measure is the single most valuable DVL test — it
validates the instrument, the parsing and the units in one shot.

If the rate is low, that is physics, not software: the DVL cannot ping again until the
previous ping returns, so the rate falls as altitude rises.

- [ ] **[R]** DVL stage: pass / fail — notes: ______________

### 4c. Camera

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py imu:=false dvl:=false foxglove:=false
```

The log should print an fps line every five seconds.

```bash
ros2 topic hz /bluerov2/camera/image_raw
ros2 topic echo /bluerov2/camera/camera_info --once
ros2 run rqt_image_view rqt_image_view   # needs X forwarding
```

| Check | Expected | Result |
|---|---|---|
| frame rate | ~30 fps, ≥ 15 | **[R]** ___ |
| `width` × `height` | matches BlueOS | **[R]** ___ |
| image looks right | not torn, not green | **[R]** ___ |
| `camera_info.k` | all zeros | **[R]** ___ |

All-zero intrinsics are **expected** — the camera is not calibrated yet, and zeros are the
documented way of saying so.

If there are no frames, go back to the Stage 2 tcpdump result. Packets arriving but no
frames means a decode problem; no packets means a BlueOS problem.

- [ ] **[R]** Camera stage: pass / fail — notes: ______________

---

## Stage 5 — Signs and frames

**This is the most important stage in the document.** Everything above proves data is
*flowing*. This proves it is *correct*. Frame and sign errors are invisible in a topic
listing and catastrophic in an estimator — they produce a trajectory that curves smoothly
in the wrong direction while every rate and every covariance looks healthy.

Nothing here has ever been checked against hardware. Take it slowly.

Bring everything up:

```bash
ros2 launch bluerov2_bringup bluerov2.launch.py
```

### 5a. Rotation signs

Move the vehicle **slowly by hand** and watch `angular_velocity` on
`/bluerov2/imu/data_raw`. ROS uses **FLU**: x forward, y **left**, z **up**.

| Motion | Expect | Result |
|---|---|---|
| Bow **up** | `angular_velocity.y` **negative** | **[R]** ___ |
| Starboard side **down** | `angular_velocity.x` **positive** | **[R]** ___ |
| Turn to **port** (left) | `angular_velocity.z` **positive** | **[R]** ___ |

Bow-up giving *negative* pitch rate is correct and surprises people: ArduPilot uses FRD
where y points right, ROS uses FLU where y points left, so the pitch axis is reversed
between them. If bow-up reads positive, the conversion is not being applied.

### 5b. Tilt signs

Hold each attitude still for a few seconds and read `linear_acceleration`:

| Attitude | Expect | Result |
|---|---|---|
| Level | (0, 0, **+9.81**) | **[R]** ___ |
| Starboard side down 90° | (0, **+9.81**, 0) | **[R]** ___ |
| Bow down 90° | (**−9.81**, 0, 0) | **[R]** ___ |

### 5c. Heading

Point the bow at each compass direction, level, and read the yaw from `imu/data`. The
driver converts ArduPilot's NED heading into **ENU**, where **0° is east** and angles
increase **counter-clockwise** — so the numbers below are not a typo.

Paste this into a shell inside the container:

```bash
ros2 topic echo /bluerov2/imu/data --field orientation --once | python3 -c "
import sys, math, yaml
q = next(d for d in yaml.safe_load_all(sys.stdin) if d)
print('ENU yaw: %.1f deg' % (math.degrees(math.atan2(
    2*(q['w']*q['z'] + q['x']*q['y']),
    1 - 2*(q['y']**2 + q['z']**2))) % 360))
"
```

(`safe_load_all` rather than `safe_load` because `ros2 topic echo` ends its output with a
`---` separator, which a single-document parse rejects. This snippet was checked against
real echo output and returns 90° / 0° / 270° / 180° for north / east / south / west.)

| Bow points | Expected ENU yaw | Tolerance | Result |
|---|---|---|---|
| North | **90°** | ± 15° | **[R]** ___ |
| East | **0°** | ± 15° | **[R]** ___ |
| South | **270°** | ± 15° | **[R]** ___ |
| West | **180°** | ± 15° | **[R]** ___ |

Use magnetic north from a phone compass, and stand well clear of the vehicle's batteries
and thrusters. A consistent offset on all four is a magnetometer calibration issue in
ArduSub. Yaw running the *wrong way* — north reading 270° instead of 90° — means the
NED→ENU conversion is inverted, which is a code bug: report it.

### 5d. DVL directions (in water)

Move the vehicle deliberately and watch `/bluerov2/dvl/velocity`:

| Motion | Expect | Result |
|---|---|---|
| Forward | `linear.x` positive | **[R]** ___ |
| To **port** (left) | `linear.y` positive | **[R]** ___ |
| Descending | `linear.z` **negative** | **[R]** ___ |

Descending giving negative z is correct: FLU has z pointing up.

---

## Stage 6 — TF and Foxglove

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link dvl_link
```

- [ ] **[R]** Tree shows: `base_link` → `imu_link`, `dvl_link`, `camera_link` → `camera_link_optical`
- [ ] `static_tf_node` warned about placeholder transforms at startup — **expected today**

Those extrinsics are unmeasured. Today is a communications test, so leave them; measuring
them is a separate job before any navigation work.

**Foxglove:** connect to `ws://<topside-laptop-ip>:8765` and import
`config/foxglove_layout.json` via Layout → Import from file.

- [ ] **[R]** Bridge connects
- [ ] **[R]** Video shows in the Image panel (topic `image_raw/compressed`)
- [ ] **[R]** IMU and DVL plots update live

If Foxglove connects and then drops during video, the Image panel is on the raw topic —
switch it to `image_raw/compressed`. Raw is ~186 MB/s and no websocket will carry it.

---

## Stage 7 — Record a reference bag

Even if things went badly, record a few minutes. A bag is worth more than any note you
write, because it can be replayed and re-examined offline.

```bash
start_mission --record     # writes into ./data
```

Get, if you can: a minute stationary, a minute of slow rotation in all three axes, and a
minute of movement in water.

- [ ] **[R]** Bag recorded, filename: ______________  size: ______

---

## Failure fingerprints

| Symptom | Most likely cause | Action |
|---|---|---|
| `No MAVLink heartbeat within 10 s` | BlueOS not forwarding to this host | Check endpoints in BlueOS; try `rov_ip:=192.168.2.2` |
| Heartbeat fine, topics slow | ArduSub ignored the rate request | Note actual rate; not a blocker |
| Accel z reads **−9.81** | frame conversion inverted | **Stop, report** — code bug |
| Bow-up gives positive pitch rate | FRD→FLU not applied | **Stop, report** — code bug |
| Yaw runs backwards | NED→ENU inverted | **Stop, report** — code bug |
| Heading off by a constant | magnetometer calibration | Recalibrate in BlueOS; not a code issue |
| `No DVL reports in the last 5 s` | wrong address, or DVL unpowered | `nc <ip> 16171` from Stage 2 |
| DVL reports arrive, velocity topic silent | no bottom lock | Normal in air; in water check altitude range |
| DVL velocity y sign flipped | `rotate_to_flu` wrong for this mount | Note it; config change |
| No camera frames, packets present | decode problem | `gst-launch-1.0 udpsrc port=5600 ! fakesink -v` |
| No camera frames, no packets | BlueOS stream not started | Fix in BlueOS video page |
| Video tears or stutters | jitter | Raise `jitter_buffer_ms` |
| Nodes cannot see each other | `ROS_DOMAIN_ID` clash | Set distinct IDs |

---

## Results sheet

```
Date: ____________   Tester: ____________   Location: ____________
Vehicle: BlueROV2 Heavy      Water: dry bench / pool / open water

STAGE 0  image builds ______   60 tests pass ______
STAGE 1  ROV ping ______ ms    DVL ping ______ ms    DVL address ____________
         video port ______     resolution ____________
STAGE 2  MAVLink pkts ______   video pkts ______     DVL JSON ______
STAGE 3  colcon build ______
STAGE 4a IMU     rate ______   |a| ______   |w| ______   mag ______   clock drift ______
STAGE 4b DVL     dry ______    in water ______   altitude err ______   fom ______
STAGE 4c CAMERA  fps ______    resolution ____________
STAGE 5a signs   pitch ______  roll ______   yaw ______
STAGE 5b tilt    level ______  starboard ______   bow-down ______
STAGE 5c heading N ______  E ______  S ______  W ______
STAGE 5d DVL dir fwd ______  port ______  down ______
STAGE 6  TF ______   Foxglove ______
STAGE 7  bag ____________

BLOCKERS:


ODDITIES (worked, but looked wrong):


```

## What to send back

1. The results sheet above.
2. The rosbag, if it is small enough — otherwise just say it exists.
3. **The full terminal output of any stage that failed**, not a summary of it. The exact
   error text is what makes a fix possible.
4. Anything in the "oddities" box. Things that *nearly* work are the most informative
   observations you can bring back.
