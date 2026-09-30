# AQUA SLAM
## 1 Introduction
AQUA-SLAM is an underwater SLAM system that integrates a Doppler Velocity Log (DVL), Inertial Measurement Unit (IMU), and stereo cameras to deliver precise and reliable pose estimation and environmental reconstruction in challenging underwater scenarios.
![offshore.gif](images/offshore.gif)
## 2 Publication

Shida Xu, Kaicheng Zhang, and Sen Wang. "AQUA-SLAM: Tightly-Coupled Underwater Acoustic-Visual-Inertial SLAM with Sensor Calibration." IEEE Transactions on Robotics, 2025. [[IEEE](https://ieeexplore.ieee.org/abstract/document/10938346)] [[PDF](https://arxiv.org/pdf/2503.11420)]

## 3 Installation
**Tested on Ubuntu20.04 and Ubuntu22.04.**
Local Host:
```bash
mkdir aqua_slam_ws/src -p
cd aqua_slam_ws/src
git clone https://github.com/DaDa0o0/AQUA_SLAM_test.git
cd ./AQUA_SLAM_test/docker
docker compose build
docker compose up -d
xhost +
docker exec -it orb_dvl2_ros_noetic bash
```
In docker container:
```bash
catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/clang++-10 DCMAKE_C_COMPILER=/usr/bin/clang-10
mkdir -p ./src/AQUA_SLAM/Vocabulary
cd ./src/AQUA_SLAM/Vocabulary
/root/.local/bin/gdown --fuzzy https://drive.google.com/file/d/1kFIh_By8wMj6AySCJ1aKjWvi7EHP7O7G/view\?usp\=sharing
cd /root/catkin_ws
source devel/setup.bash
roslaunch ORB_DVL2 blue_gx5_StructureEasy.launch
```

You should be able to see a rviz window after this step.

**Note:** We provide multiple launch files for each sequence (e.g., `blue_gx5_StructureEasy.launch` for `Structure_Easy.bag`), as some sequences use different extrinsic calibrations. Please make sure to use the corresponding launch file when running each ROS bag.

## 4 Running with Tank Dataset

### Dataset Download
Please visit [Tank Dataset](https://senseroboticslab.github.io/underwater-tank-dataset/) website to download ros bags.

Here we download **Structure_Easy.bag** for test.
![bag.png](images/bag.png)

Place the ros bag under `aqua_slam_ws/src/AQUA_SLAM_test/dataset`

### Run with a ROS bag
Open a new terminal on local host to play the rosbag:
```bash
docker exec -it orb_dvl2_ros_noetic bash
rosbag play ./src/AQUA_SLAM/dataset/Structure_Easy.bag
```
Now you should see the SLAM running in rviz.

## 5 Run with your own data
You need to modify the camera parameters and extrinsic parameters to your own setting in the YAML file under `data` folder.

### Important parameters
Here we list some parameter affect performance seriously:
- **Sensor Extrinsics**:
  - `T_dvl_c`: Transformation matrix from the DVL frame to the Camera frame.
  - `T_gyro_c`: Rotation matrix from the IMU frame to the Camera frame.

- **IMU & DVL Settings**:
  - `IMU.NoiseGyro`, `IMU.NoiseAcc`: Noise parameters for the IMU.
  - `IMU.GyroWalk`, `IMU.AccWalk`: Random walk parameters for the IMU.
  - `IMU.Frequency`: Frequency of the IMU.
  - `alpha`,`beta`: DVL Transducer orientation.

- **Initialization**:
  - `IMUInitTranslation`: The distance threshold for IMU initialization. IMU initialization will be triggered when the distance traveled by the camera exceeds this threshold.
  - `IMUInitRotation`: The rotation threshold for IMU initialization. IMU initialization will be triggered when the rotation angle of the camera exceeds this threshold.
- **Loop Closure**
  - `EnableLoopDetection`: Enable/Disable loop closure detection.

## 6 Results
![wh.gif](images/wh.gif)
## 7 Known Issues
- Currently, the system may randomly crash when running with long sequences due to some multithreading issues. We are working on fixing this problem. If you encounter a crash, please try restarting the system again.

## 8 Citation
If you find this work useful in your research, please consider citing:
```
@article{xu2025aqua,
  title={AQUA-SLAM: Tightly-Coupled Underwater Acoustic-Visual-Inertial SLAM with Sensor Calibration},
  author={Xu, Shida and Zhang, Kaicheng and Wang, Sen},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}
```

If you use the Tank Dataset, please also cite:
```
@article{xu2025tank,
  title={Tank dataset: An underwater multi-sensor dataset for SLAM evaluation},
  author={Xu, Shida and Scharff Willners, Jonatan and Roe, Joshua and Katagiri, Sean and Luczynski, Tomasz and Petillot, Yvan and Wang, Sen},
  journal={The International Journal of Robotics Research},
  pages={02783649251364904},
  year={2025},
  publisher={SAGE Publications Sage UK: London, England}
}
```