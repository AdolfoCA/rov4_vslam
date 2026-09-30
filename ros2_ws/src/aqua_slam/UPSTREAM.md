# Upstream and port notes

**Source:** https://github.com/SenseRoboticsLab/AQUA-SLAM, commit
`f190342d5d441fa1b6dfd654eff9b91d3414c116` (2026-01-09), copied without its git history.
Licence: GPL-3.0 (`LICENSE`), which also covers this directory in our repository.

> Shida Xu, Kaicheng Zhang, Sen Wang. *AQUA-SLAM: Tightly-Coupled Underwater
> Acoustic-Visual-Inertial SLAM with Sensor Calibration.* IEEE T-RO, 2025.

The upstream package is ROS 1 Noetic (catkin, `ORB_DVL2`), built on ORB-SLAM3 with
OpenCV 3. This is a port to ROS 2 Humble / OpenCV 4, run inside the bluerov2 container.
The SLAM algorithm is unchanged. To see exactly what changed, clone upstream at that
commit and diff `src/` and `include/` against this directory.

## What changed

**Build**
- catkin → ament_cmake, package renamed `ORB_DVL2` → `aqua_slam`.
- OpenCV 3.2 → the system OpenCV 4.5 (the same one `cv_bridge` uses, so there is only
  one OpenCV in the process). Removed OpenCV 3 constants renamed (`CV_BGR2GRAY` →
  `cv::COLOR_BGR2GRAY`, `CV_FONT_*`, `CV_LOAD_IMAGE_*`, `CV_REDUCE_SUM`, `opencv/cv.h`).
- `src/Optimizer.cc` (14k lines) split into `src/Optimizer_part1..5.cc` at function
  boundaries: compiling it whole needed more than 4 GB of RAM. The parts concatenated
  are the original file (compare with `cat src/Optimizer_part*.cc`, minus each part's
  repeated include block); `sortByVal()` became a file-local helper in each part.
  The g2o-heavy units (the parts, `DvlGyroOptimizer.cpp`, `G2oTypes.cc`) compile at `-O2`
  instead of `-O3`, with eager GCC garbage collection (`CMakeLists.txt`). Even so, every
  unit including the SLAM headers needs ~3.3-4 GB in cc1plus: build with one job.
- `PnPsolver` uses the OpenCV C API (`CvMat`, `cvSVD`); `opencv2/core/core_c.h` is now
  included explicitly. PCL 1.12 uses `std::shared_ptr` (`DenseMapper.cpp`).
- `Thirdparty/octomap` removed (it carried its own `package.xml` and would clash
  with ROS's octomap); the system `ros-humble-octomap` is used.
- `Thirdparty/g2o`, `Thirdparty/DBoW2`: author paths (`/home/da/...`) removed, OpenCV 4,
  libraries built into the build tree instead of the source tree, `cmake_minimum_required`
  raised to 3.5 (CMake deprecation warnings), and g2o no longer dumps every CMake
  variable into `cmake_variables.txt` in the source tree on each configure.
- Removed because they were not compiled upstream either or are ROS 1 only:
  `src/test`, `src/dvl_model`, `Viewer`/`MapDrawer` (Pangolin), `g2o_BA.cpp`,
  `lkTracker.cpp`, `stereo_dvl_ros.cpp`, `ros_stereo_DVL_tighly_rovco.cc`, the copied
  ROS 1 `cv_bridge` (`include/cv_bridge_slam`, `cv_bridge.cpp`, `rgb_colors.cpp`),
  `docker/`, `launch/*.launch`, `urdf/`, `images/`, `dataset/`.

**ROS**
- `include/ros1_compat.h`: maps the ROS 1 pieces used all over the core — `ROS_*`
  logging, `ros::Time`, `ros::Duration`, message type names — onto ROS 2, so that
  Tracking/Optimizer/LoopClosing/... stay line-for-line as upstream.
- Real ROS plumbing rewritten against rclcpp: `RosHandling` (publishers, services,
  TF), `LoopClosing`, `LKTracker`, `DenseMapper` publishers, `System` parameters. The
  core publishes through one node, set by the executable with `aqua_slam::SetNode()`.
- Topics `/AQUA_SLAM/...` → node-private `~/...`, i.e. `/aqua_slam/...`.
- Frames: `AQUA_SLAM` → parameter `map_frame` (default `aqua_slam_map`); the TF child
  `/bluerov/base_link` → parameter `body_frame` (default `aqua_slam_camera`). The pose
  is the camera's, so publishing it as our `base_link` would be wrong by the lever arm.
- Services (`std_srvs/Empty`): `/aqua_slam/save`, `/aqua_slam/load_map`,
  `/aqua_slam/calibrate`, `/aqua_slam/fullBA`.
- `/ORBSLAM3_tightly/*` and `/AQUA_SLAM/*` global parameters → node parameters
  `is_load_map`, `out_path`, `traj_path`, `map_file`.

**Node** (`src/aqua_slam_node.cpp`, was `src/ros_stereo_DVL_tighly.cc`)
- Same measurement handling (`SyncWithImu2` upstream): stereo pairing within 0.1 s,
  IMU + DVL up to the left image stamp, bad-DVL rejection (`velocity_valid`, |v| > 1 m/s),
  `TrackStereoGroDVL` with the merged gyro/DVL stream.
- DVL message: `waterlinked_a50_ros_driver/DVL` → `bluerov2_msgs/DVLReport`. Our driver
  publishes velocity in FLU; it is rotated back to the A50's FRD, the frame of
  `T_dvl_c` and the beam model (`dvl_velocity_frame`). Reports with < 4 beams are dropped
  (upstream indexed `beams[0..3]` unconditionally).
- Best-effort subscriptions (our drivers publish with the sensor-data QoS).
- Optional rectification (`rectify`, `LEFT.*`/`RIGHT.*`), as in the ORB-SLAM3 examples.
- Topics, vocabulary, settings file and debug-log directory are ROS parameters.
- The sync loop sleeps when idle (upstream busy-waited on a full core), buffers are
  read under their mutexes.

**Robustness**
- `System`: a missing settings file or vocabulary throws instead of `assert(0)`, which
  is compiled out in release builds and let the system continue with garbage.
- `LocalMapping::GetTravelDistance`: the relative rotation between keyframes comes from
  `CV_32F` poses and is orthonormal only to ~1e-7; Sophus 1.22's `SO3(Matrix3d)` checks
  to 1e-10 and aborts the process on the first real keyframe. It is now built with
  `SO3::fitToSO3`. Found with synthetic stereo/IMU/DVL data.

## Known upstream issues, not fixed here
- The README warns the system "may randomly crash when running with long sequences due
  to some multithreading issues".
- Image queues are unbounded: if tracking runs slower than the cameras, memory grows.
