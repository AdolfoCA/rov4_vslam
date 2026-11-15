# Calibration — what a bag compatible with AQUA-SLAM needs

> **Note of 14/11/2026:** the data are now on the KODAK SSD (`/media/andrea/KODAK/Andrea/`). The bags of 15/10, 20/10, 23/10, 25/10 (kalibr `sessione2/3`), the tests of 26/10, `prova_pipeline_30-10` and the conversions `01_November` / `02_November` have been deleted: the paths that mention them below are historical.


Goal: get a ROS 1 `.bag` that AQUA-SLAM (`stereo_dvl2`) can use like
`Structure_Easy.bag`, plus a yaml with the parameters of **our** ROV.

Starting point (checked on `data/dive02`, 13/10/2026):

| Sensor | Recorded topic | Rate | Status |
|---|---|---|---|
| IMU | `/bluerov2/imu/data_raw` (+ `mag`, `pressure`, `temperature`) | 100 Hz | OK, timestamps already good |
| DVL | `/bluerov2/dvl/report` (`bluerov2_msgs/DVLReport`) | 5 Hz | OK, timestamps on arrival |
| Cameras | `/bluerov2/multicam/aux_{left,right}/image_raw/compressed` | 30 Hz, 960x540 JPEG | OK, **not calibrated** (`camera_info` all zero) |

Rates and formats are **not blocking**: they are handled with the yaml and the conversion
script. The real work is the calibration, sensor by sensor.

Recommended order: **0 → 1 → 2 → 3 → 4 → 5 → 6 → 7**. Each step lists its
dependencies.

---

## 0. ROS 2 → ROS 1 conversion script (prerequisite for everything)

Needed for AQUA-SLAM, but also for Kalibr and `allan_variance_ros`, which are ROS 1.

- [x] **For Kalibr (25/10):** `data/calibrazione/stereo_e_imu/02_converti_ros1.py` converts
      images (original JPEG, `CompressedImage`) and IMU into a ROS 1 `.bag` with the `rosbags`
      library, without ROS 1. Same topic names as the original. Checked with the Kalibr
      readers. The bags are on the SSD, `KODAK/Andrea/kalibr/`.
- [ ] **For AQUA-SLAM:** the same script, to be extended with the DVL, the image format and the
      rectification (items below).
- [ ] Convert MCAP/CDR into a ROS 1 `.bag`.
- [ ] DVL: `bluerov2_msgs/DVLReport` → `waterlinked_a50_ros_driver/DVL` on the topic **`/dvl/data`**
      (name fixed in the AQUA-SLAM code).
  - `beam_valid` → `valid`, `format` → `form`, `time_delta` → `time` (in ms), covariance dropped.
  - Undo the FLU rotation (x, −y, −z) **or** account for it in `T_dvl_camera` (step 5).
- [ ] Images: rewrite `format` as `"bgr8; jpeg compressed bgr8"`.
- [ ] (Optional) DVL timestamps corrected from `time_of_validity` (see step 6).
- [ ] (Optional) `/depth/data` from the pressure — AQUA-SLAM does not read it.

Images and IMU need no renaming: the names go in the yaml (`LeftImgTopic`,
`RightImgTopic`, `ImuTopic`).

---

## 1. IMU — noise parameters (stationary recording)

**What it gives:** noise density and random walk of accelerometer and gyroscope
(`IMU.NoiseGyro`, `IMU.NoiseAcc`, `IMU.GyroWalk`, `IMU.AccWalk`). Needed by Kalibr (step 4)
and by the AQUA-SLAM yaml.

**Where:** out of the water is fine, even better (no waves, no currents).

Step by step:
- [x] ROV resting **on the ground** on a stable surface, not on a table, away from
      vibrations (pumps, compressors, people walking by).
- [x] Thrusters **disarmed**.
- [x] ~~Switch the ROV on and wait 10–15 min for the temperature to settle.~~ Not needed:
      the warm-up is discarded in the analysis (see the result below).
- [x] Record only the `imu` group.
- [x] Record **at least 1 hour, better 2–3 hours**.
- [x] Nobody touches the ROV during the recording.
- [x] Check `imu/temperature` in the bag.
- [x] Allan deviation analysis (our own script instead of `allan_variance_ros`, same method).
- [x] Multiply the values by a safety factor (×10) before using them in Kalibr / ORB-SLAM3.

### Result (T0, 17/10/2026) ✅

**Recording:** `/media/andrea/KODAK/Andrea/imu_fermo` (external SSD), 15:11 → 17:30,
**138 min**, 829 145 samples at 100 Hz, 7 missing (0.001%). ROV on the ground, still, disarmed.

**IMU temperature:** 30 → 57.5 °C. It rises fast in the first 30 min, then slowly; almost
flat after ~100 min. The bias follows the temperature: gyroscope z from −0.40 to −0.02 °/s,
accelerometer y from 0 to +0.20 m/s². Only the **warm part (60–138 min)** is used in the analysis.

**Values** (warm part, worst axis, then ×10):

| Parameter | Measured | **To use** | Structure_Easy |
|---|---|---|---|
| `IMU.NoiseGyro` / `gyroscope_noise_density` [rad/s/√Hz] | 7.1e-5 | **7.1e-4** | 1e-3 |
| `IMU.NoiseAcc` / `accelerometer_noise_density` [m/s²/√Hz] | 8.2e-4 | **8.2e-3** | 1e-2 |
| `IMU.GyroWalk` / `gyroscope_random_walk` [rad/s²/√Hz] | 1.1e-5 | **1.1e-4** | 1e-5 |
| `IMU.AccWalk` / `accelerometer_random_walk` [m/s³/√Hz] | 1.9e-4 | **1.9e-3** | 1e-4 |

- **Noise** (N, read at τ = 1 s): the same over the whole recording, warm-up included.
- **Random walk** (K): +½ line kept **above** the Allan curve for τ ≥ 20 s. At long times
  the curve rises more steeply than +½ (slow thermal drift): a fit on a short stretch
  would give about half, too optimistic. With only 90–138 min the values are equal or lower.
- Noise of the same order as Structure_Easy; random walk 10–20× larger (low-cost IMU,
  no temperature control).
- The gyroscope y axis looks less noisy: it is the 1 mrad/s quantisation, not a better
  sensor. That is why the worst axis is used.
- Below τ ≈ 0.05 s the curves flatten (probably an internal ArduSub filter): irrelevant.
- These are **starting values**: if the SLAM with IMU turns out unstable, they are the first to
  retune (usually upwards).

**Files** in `data/calibrazione/imu/`:
- `imu_kalibr.yaml` — IMU config for Kalibr (step 4), ready;
- `aqua_slam_imu.yaml` — `IMU.*` lines for the AQUA-SLAM yaml (step 7);
- `allan_gyro.png`, `allan_acc.png`, `temperatura_bias.png` — plots;
- `risultati_allan.txt` — all the numbers per axis and per time span;
- `imu_allan.py` — the analysis script, to redo it.

Notes:
- 100 Hz and a gyroscope quantised at 1 mrad/s are **not blocking**. Raising the IMU rate is a
  change on the robot side (ArduSub): only after discussing it.
- The IMU timestamps are regular (sensor time `time_usec` + offset filter), but Kalibr
  (31/10) finds them **about 16 ms late** on the real motion: see steps 4 and 6.

---

## 2. Stereo cameras — intrinsics and baseline

**What it gives:** fx, fy, cx, cy, distortion of each camera, and the left↔right
transform (baseline → `Camera.bf`).

### Result (Kalibr, 31/10/2026) ✅ — stereo pair, new right camera

Recordings of 30/10 in the pool with our pipeline (capture timestamps, exact pairs):
`KODAK/Andrea/Pool/30_October/stereo_1`, `stereo_2`, `stereo_3` (2, 3.5 and 3.5 min; target
still, ROV driven with the joystick to cover the whole image), merged into one ROS 1 bag
(`KODAK/Andrea/kalibr/stereo_30-10.bag`). Coverage checked beforehand with
`stereo_e_imu/04_controllo_aprilgrid.py`. Kalibr `kalibr_calibrate_cameras`, pinhole-radtan,
`--bag-freq 2`: 1738 views extracted, **126 pairs kept**. Files and command:
`data/calibrazione/stereo_e_imu/kalibr_stereo_30-10/` (`lancia.sh`).

| | left (cam0) | right (cam1) |
|---|---|---|
| fx, fy [px] | 726.10 / 726.83 (±0.4) | 722.64 / 723.16 (±0.4) |
| cx, cy [px] | 478.90 / 271.69 (±0.5) | 479.85 / 277.16 (±0.5) |
| k1, k2, p1, p2 | −0.2912, 0.1031, −0.00028, −0.00010 | −0.2898, 0.1013, 0.00030, −0.00028 |
| reprojection error (std x / y) | **0.13 / 0.12 px** | **0.14 / 0.13 px** |

- **Baseline 22.02 cm** (±0.15 mm): right relative to left −22.01 / +0.25 / +0.36 cm.
- **Relative rotation 2.1°**: 2.0° of pitch, 0.56° of yaw, 0.34° of roll. The 2.0° of
  pitch explain the raw vertical offset of about −26 px measured at the harbour on 27/10
  (726 px · tan 2.0° = 25 px). The estimate of 26/10 (4° and 3.7°) was wrong.
- Left consistent with the calibration of 25/10 (same focal length and distortion; cx 478.9 against
  482.3, i.e. 3.4 px: prefer the new value, better coverage).

Camchain (to pass to step 4 and to the AQUA-SLAM yaml):
`kalibr_stereo_30-10/stereo_30-10-camchain.yaml`.

### History (up to 26/10/2026)

| Date | Calibration | Status |
|---|---|---|
| 15/10 | OpenCV, pool, `data/calibrazione/camera/out/` | intrinsics valid for the **left** camera |
| 23/10 | "hybrid" OpenCV (intrinsics 15/10, extrinsics 23/10), `camera/sessione_23-10_estrinseci/` | **superseded for the right camera**: camera replaced on 26/10 |
| 25/10 | Kalibr, left camera only, `stereo_e_imu/kalibr_camera_sx/` | reprojection error about 0.18 px (OpenCV 1.6 px) |
| **26/10** | **right camera replaced** (the old one was not sharp: an optical problem, proven by swapping the streams on 23/10) | ❌ **to redo in the water**, with the target |

Checks on the new camera (26/10, in air, `KODAK/Andrea/prova_camere_aria`, `prova_lab_aria`):
- right/left sharpness **0.91-0.95** (old camera: 0.35-0.50), identical `v4l2-ctl` settings;
- mounted **about 4° more tilted vertically and about 3.7° horizontally** than the old one
  (raw vertical offset −35 px against +16 px). The hardware stays like this: the rectification
  compensates for it (checked in air: 0.0 px offset after rectification).
- The camera mount moves when the ROV is handled (23/10: from 0.0 to +1.8 px in one
  afternoon): **check it at every outing** with `camera/scripts/08_check_dy.py`.
- All the details: `data/Verifica_ORB/fase2_analisi/RESOCONTO_PORTO_20-10.md` (sections 10-11),
  `RESOCONTO_PARTE2_24-10.md`, `data/calibrazione/stereo_e_imu/RESOCONTO_CALIBRAZIONI_26-10.md`.

Step by step:
- [ ] Check that aux_left and aux_right **look in the same direction and overlap**
      enough to be a stereo pair.
- [ ] Check that the calibration videos were **taken in the water** (the flat port
      refracts: intrinsics in air are not valid).
- [ ] Check the **video resolution**: it must be the one of the stream (**960x540**),
      with the same crop or scale.
  - If the videos are at native resolution, rescale the intrinsics.
  - If the stream is a crop (not a downscale), the values cannot be transferred.
- [ ] Check that the left and right videos are **synchronised** with each other.
- [x] Calibrate (Kalibr `kalibr_calibrate_cameras`), pinhole + radtan model (31/10).
- [x] Check the reprojection error: 0.12-0.14 px.
- [ ] Also save the result as `camera_info_url` for the camera nodes (today K is all zero).
- [ ] Rectification (R1/R2/P1/P2) from the new calibration, for AQUA-SLAM and for the vertical offset.

Open questions (answered on 30-31/10): videos in the water, 960x540 as the stream, synchronised
pairs (capture timestamps).

---

## 3. (No hardware) Check the hardware synchronisation of the cameras

- [x] **Checked on 26/10: the aux cameras fire together**, offset **below 1-2 ms**
      (`frame_sync = 1` on all cameras). Measurement: object moved vertically in front of the
      cameras; the left-right vertical offset does not grow with the speed (5466 moving
      points, flat line: +0.1/+0.3 ms). `KODAK/Andrea/prova_sync/`.
  - The ±15 ms offsets seen in the arrival timestamps (and the 25/10 hypothesis of cameras out of
    step after a reboot) came from the network/driver path, not from the shutters.
  - With the RTP timestamps (step 6) a **constant** offset of about −8.6 ms remains between left
    and right, and it is not real: it comes from the encoding order on the camera computer. The
    pairs are formed correctly; for the SLAM the pair can take the left timestamp.
- [x] **30/10: cause found and fixed.** Kernel timestamps of the two aux cameras: they capture
      within **±14 µs**. The offset (−8.6 / −14.8 ms) and the one-frame pairing changes of the
      harbour on 27/10 come from the Blue Atlas software, which resamples every camera at 30 Hz
      with the Jetson clock (the cameras run at 30.0028 Hz): about every 355 s each camera repeats
      or drops a frame, at different times. Solution: our own pipeline for the aux cameras
      (`multicam sync-start`, `aux_pipeline:=sync`), with capture timestamps and exact pairing.
      Details: `data/prova_pipeline/RESOCONTO_SINCRONIA_CAMERE_30-10.md`.
  - Data recorded with Blue Atlas: the left/right pairing can be wrong by one frame at
    times; check it in windows with `stereo_e_imu/03_accoppiamento_sx_dx.py`.

---

## 4. Camera ↔ IMU — extrinsics and time offset (Kalibr)

**What it gives:**
- `T_imu_camera` (where the camera is relative to the IMU);
- **delay of the cameras relative to the IMU** (`timeshift_cam_imu`). Essential: ORB-SLAM3
  does not estimate it by itself.

**Depends on:** step 0 (conversion), 1 (IMU noise), 2 (intrinsics).

### Result (Kalibr, 31/10/2026) ✅ — both cameras, capture timestamps

Recordings of 30/10 in the pool, our pipeline: `Pool/30_October/stereo_imu_1`, `_2`, `_3`
(target still, ROV driven with the joystick; imu_1 slow with the target always in view, imu_2 and
imu_3 fast). Checked beforehand with `04_controllo_aprilgrid.py` and `05_controllo_camimu.py`,
converted one per bag. Kalibr `kalibr_calibrate_imu_camera` with the stereo camchain of step 2 and
`imu.yaml`, `--bag-freq 15`, `--timeoffset-padding 0.1`, `--imu-delay-by-correlation`, spans
`--bag-from-to` 13-155 / 2-138 / 1-179 s. Files and command:
`data/calibrazione/stereo_e_imu/kalibr_imu_camera_30-10/` (`lancia.sh`).

Position in the IMU frame (x forward, y left, z up) [cm]:

| | imu_1 | imu_2 | imu_3 | mean | measured by hand |
|---|---|---|---|---|---|
| left camera | 19.9 / 13.0 / −22.7 | 20.0 / 12.5 / −22.5 | 20.5 / 12.7 / −22.6 | **20.1 / 12.7 / −22.6** | 20 / 14 / −23 |
| right camera | 19.8 / −9.0 / −22.1 | 19.9 / −9.5 / −21.9 | 20.2 / −9.3 / −22.0 | **20.0 / −9.3 / −22.0** | |

| | imu_1 | imu_2 | imu_3 |
|---|---|---|---|
| downward tilt left / right | 29.58° / 27.60° | 29.60° / 27.63° | 29.69° / 27.72° |
| **delay** left / right (t_imu = t_cam + delay) | 15.6 / 15.9 ms | 16.3 / 16.4 ms | 15.5 / 15.6 ms |
| reprojection error (median) left / right | 0.21 / 0.23 px | 0.24 / 0.25 px | 0.29 / 0.30 px |

- **Repeatability:** positions within 0.5 cm, rotations within 0.39°, delay within 0.8 ms.
  Normalised gyroscope and accelerometer residuals below 1 (consistent with `imu.yaml`).
- **Delay about 16 ms** (it was 82 ms with the arrival timestamps: almost all of it was the image
  latency). Now the cameras carry the capture instant, so the 16 ms are the delay of the
  **IMU** timestamps (ArduSub → MAVLink → PC chain). The same for the two cameras within 0.3 ms.
- The vertical axis is the least excited in the recordings (the ROV moves little up and down),
  but the height agrees between the three estimates within 2 mm and with the hand measurement
  within 0.4 cm.

Left camera (**not rectified**) → IMU, imu_1 (the recording with the most views of the target):
```
[[-0.00270, -0.49374,  0.86960,  0.19944],
 [-0.99981, -0.01563, -0.01197,  0.12992],
 [ 0.01950, -0.86947, -0.49360, -0.22721],
 [ 0.0,      0.0,      0.0,      1.0    ]]
```

### Previous result (Kalibr, 25/10/2026) — left camera only, arrival timestamps

Kalibr installed on the PC (ROS Noetic) in `~/rov4_vslam/kalibr_ws/`; procedure and how it works in
`data/calibrazione/stereo_e_imu/SCHEMA_KALIBR.md`. Recordings of 23/10 in the pool, ROV driven
with the **joystick** (not moved by hand): session 2 (15:19) and session 3 (15:33). Left camera
only: the geometry between the cameras was shifting during the recordings.

| | Session 2 | Session 3 | Measured by hand |
|---|---|---|---|
| Left camera in the IMU frame (x forward, y left, z up) | 20.5 / 12.6 / −25.1 cm | 20.6 / 12.8 / −23.9 cm | 20 / 14 / −23 cm |
| Downward tilt | 29.6° | 29.5° | ~29° |
| Camera delay (arrival timestamps) | 81.9 ms | 82.2 ms | — |
| Reprojection error (median) | 0.32 px | 0.37 px | |

The two sessions agree within 0.3°, 1.2 cm and 0.2 ms. Files:
`data/calibrazione/stereo_e_imu/kalibr_imu_camera_sx/`.

Camera (left, **not rectified**) → IMU, session 2 (inverse of Kalibr's `T_cam_imu`):
```
[[-0.00783, -0.49452,  0.86913,  0.20473],
 [-0.99985, -0.00947, -0.01439,  0.12550],
 [ 0.01535, -0.86911, -0.49437, -0.25069],
 [ 0.0,      0.0,      0.0,      1.0    ]]
```

To do:
- [x] **Delay re-estimated (31/10): about 16 ms**, with the capture timestamps (above).
- [x] The 16 ms are compensated in the IMU driver (step 6), since 31/10.
- [ ] `T_gyro_c` for AQUA-SLAM: compose the 31/10 matrix with `R1` of the rectification of the
      new stereo calibration (rectified left camera).
- [ ] If the left camera or its mount are touched, redo the calibration.

Preparation:
- [ ] Print an **aprilgrid** (not a checkerboard) and glue it on a rigid, flat board
      (forex/aluminium), laminated for the water. Measure the real size of the tags.
- [ ] Pool or tank, with plenty of light (short exposure needed, no motion blur).

Recording:
- [ ] **Same pipeline and same parameters as the dive**: same resolution, same camera node,
      same `jitter_buffer_ms`. The measured delay is valid only for that configuration.
- [ ] Active groups: `imu`, `aux_left`, `aux_right`.
- [ ] **Target still, ROV moving** (moved by hand in front of the target).
- [ ] 60–90 s of motion exciting **all 6 axes**: roll, pitch, yaw,
      forward/backward, up/down, right/left.
- [ ] Decisive motion but no blur in the images; target **always visible from both**
      cameras.
- [ ] Make 2–3 recordings, to have a spare and to compare the results.

Processing:
- [ ] Convert (step 0), run `kalibr_calibrate_imu_camera` with the intrinsics fixed
      from step 2 and the noise from step 1.
- [ ] Check that the recordings give similar extrinsics and delay.
- [ ] Put `T_imu_camera` in the yaml and the delay in `timestamp_offset_s` of the camera
      node (or correct it in post-processing).

---

## 5. DVL ↔ camera — position of the DVL

**What it gives:** `T_dvl_camera`.

- [x] Measure the position and orientation of the DVL relative to the IMU (tape measure) — see below.
- [x] Combine with camera↔IMU to get `T_dvl_c` — see below.
- [x] Convention: native A50 frame (x forward, y right, z down), as in Structure_Easy; the
      conversion (step 0) brings the velocities of our driver (FLU) into that frame.
- [ ] Check the DVL alignment on the data (P5, or DVL velocity vs camera motion).
- [ ] Check the `speed_of_sound` set on the DVL for the water of the recording day
      (fresh/salt). A change on the robot side: only after discussing it.

### Extrinsics measured by hand (18/10/2026)

Measured with a tape measure, **origin = IMU (Navigator board)**, ROV axes: x forward, y
left, z up (sheet from `SCHEDA_MISURE.md`).

| Sensor | x forward | y left | z up | Orientation |
|---|---|---|---|---|
| IMU (Navigator) | 0 | 0 | 0 | board mounted vertically, **already compensated by ArduSub**: in T0 gravity is all on +z → IMU↔ROV rotation = 0 |
| aux_left camera (lens centre) | +20 cm | +14 cm | −23 cm | tilted **~28–30° downwards** (by eye), 29° used |
| DVL (centre of the transducer face) | −9 cm | +3 cm | −32 cm | points down, A50 x axis forward, `mounting_rotation_offset` = 0 |

AQUA-SLAM (`underwater_orbslam3_blue_gx5_short.yaml`) reads **`T_gyro_c`** and **`T_dvl_c`**
(`Tbc` is ignored). Convention checked on the Structure_Easy values: camera → sensor
matrix, columns of the rotation = camera axes in the sensor frame, translation = camera
position in the sensor frame. The camera is the **rectified left** one (including R1 of the
stereo rectification, ~1°): it looks 28° below the horizon.

```yaml
T_gyro_c: !!opencv-matrix      # camera (left, rectified) -> IMU
  rows: 4
  cols: 4
  dt: f
  data: [ -0.01983, -0.47019,  0.88234,  0.20000,
          -0.99969, -0.00407, -0.02464,  0.14000,
           0.01518, -0.88256, -0.46996, -0.23000,
           0.0,      0.0,      0.0,      1.0 ]

T_dvl_c: !!opencv-matrix       # camera (left, rectified) -> DVL (A50 frame)
  rows: 4
  cols: 4
  dt: f
  data: [ -0.01983, -0.47019,  0.88234,  0.29000,
           0.99969,  0.00407,  0.02464, -0.11000,
          -0.01518,  0.88256,  0.46996, -0.09000,
           0.0,      0.0,      0.0,      1.0 ]
```

Check: in the DVL frame the camera is 29 cm forward, 11 cm to the left, 9 cm higher
(20−(−9), 14−3, −23−(−32)).

| | Translation | Rotation | Delay |
|---|---|---|---|
| Camera↔IMU | ✅ **Kalibr 31/10**, both cameras, three recordings within 0.5 cm; hand measurement within about 1 cm | ✅ **Kalibr 31/10** (step 4), within 0.4° | ✅ **about 16 ms** (IMU timestamps late); compensated in the IMU driver since 31/10 (step 6) |
| DVL↔camera | ✅ measured | ✅ assumed aligned, to be checked | — |

Done with Kalibr on 25/10 (step 4). `T_gyro_c` and `T_dvl_c` in the yaml still have to be
updated with the Kalibr rotation and the `R1` of the new stereo calibration.

---

## 6. Timestamps

| Sensor | Today | What to do |
|---|---|---|
| IMU | ✅ sensor time + filter | nothing |
| DVL | on arrival (TCP); the bag contains `time_of_validity` (A50 clock not synchronised) | **post-processing**: apply to `time_of_validity` the same filter as the IMU in the step 0 script |
| Cameras | ✅ **since 26/10: RTP timestamps** (`timestamp_source: "rtp"` for aux_left and aux_right in `bluerov2.yaml`) → exact 33.33 ms intervals, zero jitter. **Since 30/10: our pipeline** (session default) → **capture instant** in real time (`timestamp_source: capture`), Jetson clock synchronised to the PC with chrony, pairs with identical timestamps | camera-IMU delay from Kalibr → `timestamp_offset_s` |

Cameras:
- [x] **RTP timestamps in the driver (26/10).** Checks done beforehand:
  - capture of the network packets (60 s): the RTP advances by **exactly 3000 ticks (33.333 ms)**
    per frame, no frame lost, follows real time within a few ppm; the arrival on the PC has
    3-4 ms of jitter. `KODAK/Andrea/prova_rtp/rtp_60s.pcap`.
  - with the new driver (40 s): intervals of **exactly 33.33 ms**, jitter **0.00 ms** (before
    4-6 ms). `KODAK/Andrea/prova_rtp/driver_rtp_40s`.
  - How it works: `rtpjitterbuffer mode=none` (PTS from the RTP timestamps only), PTS brought
    into the ROS clock with an offset that follows the minimum arrival delay and adapts slowly
    to drifts (`rtp_offset_drift_us_per_s`). Diagnostic topic per camera:
    `/bluerov2/multicam/<camera>/time_reference` (arrival and RTP time of every frame).
  - Reconstructing the times afterwards on bags recorded with arrival timestamps was
    tried and discarded (25/10): with those data it does not work. Old bags keep the arrival
    timestamps.
- [x] **Our pipeline for the aux cameras and updated driver (30/10)**, not yet committed.
      Report: `data/prova_pipeline/RESOCONTO_SINCRONIA_CAMERE_30-10.md`.
  - `session_start` starts our pipeline by itself (`multicam sync-start`, instead of the Blue
    Atlas camera service: `bluerov2_bringup/jetson/aux_sync_pipeline.py`); `session_stop` stops it
    and shuts the Jetson down. Nothing at boot: after a Jetson power cycle Blue Atlas is back (the
    ROV is shared with a colleague).
  - **Clocks:** chrony on the PC (server) and on the Jetson (client), installed with
    `multicam timesync-setup` (`TOPSIDE_SETUP.md` 2g). Offset checked at +0.4 ms (measurable to
    ±1.75 ms over the cable); check: `multicam timesync-check`.
  - **Capture timestamps:** the pipeline runs on real time, so the RTP timestamp is the capture
    instant and the driver uses it directly (`timestamp_source: capture`), without estimating
    latencies. Pairs within 0.27 ms, 0 steps off the grid, cameras at exactly 30 Hz. If the Jetson
    clock is not synchronised, the driver reports it and falls back to the `rtp` method (minimum
    latency, 100 µs/s rise, an error of a few ms).
  - With `aux_pipeline:=blueatlas` the offset of about 14 ms stays as it was (Andrea's choice).
- [x] **Camera-IMU delay with Kalibr (31/10): about 16 ms** (step 4); t_imu = t_cam + 16 ms,
      i.e. the IMU timestamps are 16 ms late on the real motion.
- [x] **Compensated in the IMU driver (31/10):** `timestamp_offset_s: -0.016` for `imu_node` in
      `bluerov2.yaml` (valid for all the IMU topics, from the next session).
- [ ] Bags recorded **before** (up to and including 30/10): the IMU timestamps are still 16 ms
      late. Correct them in the conversion or in AQUA-SLAM if those bags are used.
- [ ] The driver still loses a few frames when the PC cannot keep up (26/10: 4 gaps of
      2-3 frames in 40 s). The timestamps stay correct.
- [ ] On 26/10 the jitter of the IMU timestamps was 1.3-1.7 ms (23/10: 0.2-0.3 ms): check it
      again on the harbour data (`stereo_e_imu/01_controllo_tempi.py`).

---

## 7. Yaml and bag for AQUA-SLAM

AQUA-SLAM used: the original ROS 1 version in `~/aqua_slam_ws/src/AQUA-SLAM` (container
`orb_dvl2_ros_noetic`, manual `~/aqua_slam_ws/src/MANUALE_AQUA_SLAM.md`). **Not** the ROS 2 copy in
`ros2_ws/src/aqua_slam` (with `COLCON_IGNORE`, never used).

**Single script:** `data/aqua_slam/converti_per_aqua.py` (on the host). From the ROS 2 recording it writes:
- the ROS 1 bag in `~/aqua_slam_ws/src/AQUA-SLAM/dataset/<name>.bag` (the only folder the
  container sees);
- the yaml `~/aqua_slam_ws/src/AQUA-SLAM/data/bluerov2_rov4.yaml`.

What it translates:
- **IMU** `data_raw`: timestamps moved earlier by `--ritardo-imu` (default 16 ms; **0 for the
  recordings since 31/10**, already corrected in the driver).
- **Cameras:** rectified with the Kalibr stereo calibration (the ROS 1 node does not rectify: the
  launch flag is ignored), JPEG quality 95, on `/bluerov2/multicam/aux_*/image_rect/compressed`.
- **DVL:** `bluerov2_msgs/DVLReport` → `waterlinked_a50_ros_driver/DVL` on `/dvl/data` (topic
  fixed in the code), velocity brought into the A50 frame (FRD), beams unchanged.

Yaml generated from the calibrations, starting from `underwater_orbslam3_blue_gx5_short.yaml`:
- [x] `Camera.width/height/fps` 960 / 540 / 30.
- [x] `Camera.fx/fy/cx/cy` of the rectified left camera (745.63 / 492.44 / 276.04: the rectification
      with `alpha=0` crops the borders), distortion 0, `Camera.bf` 164.17 (baseline 22.02 cm) ← step 2.
- [x] `T_gyro_c` ← step 4: mean of the three camera-IMU calibrations of 30/10, composed with R1 of the rectification.
- [x] `T_dvl_c` ← rotation from `T_gyro_c`, DVL position measured by hand (step 5).
- [x] `IMU.Frequency: 100`, noise from `data/calibrazione/imu/aqua_slam_imu.yaml` ← step 1.
- [x] Topics: `/bluerov2/multicam/aux_*/image_rect`, `/bluerov2/imu/data_raw`.
- [x] First converted bag (31/10): `AQUA_SLAM_test_2` (pool 30/10), checked with
      `check_bag_aqua.py`: all OK, the only warning is a 0.78 s gap in the DVL.
- [ ] AQUA-SLAM test (done by Andrea).

**Open question — record directly in the AQUA-SLAM format**, so that no conversion is needed anymore:
- [ ] DVL: publish (also) `waterlinked_a50_ros_driver/DVL` on `/dvl/data`, velocity in FRD;
- [ ] cameras: publish the rectified images (compressed), with the calibration of 31/10;
- [x] IMU: 16 ms delay already corrected in the driver (31/10);
- [ ] it is still ROS 2 → ROS 1 anyway: without conversion it would need `ros1_bridge` or the ROS 2
      port of AQUA-SLAM (see the manual, section "Real-time: ROS 1 o ROS 2").

---

## Data collection day

- [ ] Switch the ROV on **during the preparation** (launch, checks, DVL calibration): there is no
      need to keep it on idle for an hour. The IMU bias changes while the electronics
      warm up (in T0: gyroscope z from −0.40 to −0.12 °/s in the first 30 min), but the SLAM
      estimates the bias by itself during the recording: the drift only has to be slow, and it is.
- [ ] DVL gyro calibration (`dvl_calibrate`) after the preparation, right before
      recording.
- [ ] Same configuration as the Kalibr recording (step 4).
- [ ] `multicam timesync-check` passes (Jetson clock synchronised), then `session_start`
      (our pipeline for the aux cameras by default); at the end of the day `session_stop`.
- [ ] Active groups: `imu`, `dvl`, `aux_left`, `aux_right` (+ `tf`, it costs nothing).
- [ ] DVL with bottom lock (`velocity_valid: true`) — check during the dive.
- [ ] At the beginning at least **3 m of translation** and a few rotations to initialise the IMU
      (`IMUInitTranslation: 3.0`).
- [ ] Smooth motion, good light, a textured structure in the field of view.
- [ ] Convert with the script of step 0 and run AQUA-SLAM with the yaml of step 7.
