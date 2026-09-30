"""AQUA-SLAM on the BlueROV2: multi-camera top pair + DVL-A50 + Navigator IMU.

Start the drivers first (session_start.launch.py or bluerov2.launch.py), then:

    ros2 launch aqua_slam aqua_slam.launch.py

On a recorded bag instead of the live vehicle:

    ros2 launch aqua_slam aqua_slam.launch.py use_sim_time:=true
    ros2 bag play data/dive01 --clock          # second shell

Arguments:
    settings:=<file>          AQUA-SLAM settings (default config/bluerov2_multicam_top.yaml)
    namespace:=bluerov2       namespace the drivers run in (input topics are under it)
    left:=aux_left            multi-camera used as the left image
    right:=aux_right          multi-camera used as the right image
    image_transport:=raw      raw | compressed (compressed for bags recorded without raw)
    rectify:=false            undistort + rectify with LEFT.* / RIGHT.* from the settings
    dvl_velocity_frame:=flu   flu if bluerov2_dvl has rotate_to_flu (default), else frd
    vocabulary:=<file>        ORB vocabulary (default /opt/aqua_slam/Vocabulary/ORBvoc.txt)
    log_dir:=                 directory for the per-frame debug log (empty = off)
    max_image_queue:=4        images waiting per camera; older ones are dropped when
                              tracking is slower than the cameras (keeps RAM bounded)
    traj_path:=<file>         where `ros2 service call /aqua_slam/save std_srvs/srv/Empty`
                              writes the keyframe trajectory
    use_sim_time:=false

Outputs (all under /aqua_slam/): orb_pose, orb_odom, orb_path, camera_pose,
integration_path, sparse_map, markers, img_with_info, and the TF
aqua_slam_map -> aqua_slam_camera. Map, paths and markers appear after the DVL-gyro
initialisation, i.e. once the vehicle has moved IMUInitTranslation metres.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_settings = PathJoinSubstitution(
        [FindPackageShare("aqua_slam"), "config", "bluerov2_multicam_top.yaml"]
    )
    ns = LaunchConfiguration("namespace")
    left = LaunchConfiguration("left")
    right = LaunchConfiguration("right")

    arguments = [
        DeclareLaunchArgument("settings", default_value=default_settings),
        DeclareLaunchArgument("namespace", default_value="bluerov2"),
        DeclareLaunchArgument("left", default_value="aux_left"),
        DeclareLaunchArgument("right", default_value="aux_right"),
        DeclareLaunchArgument("image_transport", default_value="raw"),
        DeclareLaunchArgument("rectify", default_value="false"),
        DeclareLaunchArgument("dvl_velocity_frame", default_value="flu"),
        DeclareLaunchArgument(
            "vocabulary", default_value="/opt/aqua_slam/Vocabulary/ORBvoc.txt"
        ),
        DeclareLaunchArgument("log_dir", default_value=""),
        DeclareLaunchArgument("max_image_queue", default_value="4"),
        DeclareLaunchArgument(
            "traj_path", default_value="/home/rosdev/data/aqua_slam_keyframes.txt"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    node = Node(
        package="aqua_slam",
        executable="aqua_slam_node",
        name="aqua_slam",
        output="screen",
        parameters=[
            {
                "vocabulary_path": LaunchConfiguration("vocabulary"),
                "settings_path": LaunchConfiguration("settings"),
                "left_image_topic": ["/", ns, "/multicam/", left, "/image_raw"],
                "right_image_topic": ["/", ns, "/multicam/", right, "/image_raw"],
                "imu_topic": ["/", ns, "/imu/data_raw"],
                "dvl_topic": ["/", ns, "/dvl/report"],
                "image_transport": LaunchConfiguration("image_transport"),
                "rectify": ParameterValue(LaunchConfiguration("rectify"), value_type=bool),
                "dvl_velocity_frame": LaunchConfiguration("dvl_velocity_frame"),
                "log_dir": LaunchConfiguration("log_dir"),
                "max_image_queue": ParameterValue(
                    LaunchConfiguration("max_image_queue"), value_type=int
                ),
                "traj_path": LaunchConfiguration("traj_path"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
