cmake_minimum_required(VERSION 3.15)
project(ba_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MavlinkFishiMSGS.msg"
  "msg/PlaneWithVariance.msg"
  "msg/PredictionResults.msg"
  "msg/RcPWMRaw.msg"
  "msg/MotionEstimate.msg"

  "srv/AttachRecordingConsumer.srv"
  "srv/AttachStreamingConsumer.srv"
  "srv/NewAttachStreamingConsumer.srv"
  "srv/CreateLogDir.srv"
  "srv/DetachConsumer.srv"
  "srv/GetExposure.srv"
  "srv/GetLogDir.srv"
  "srv/SetBrightness.srv"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/StopStreaming.srv"
  "srv/AttachPointCloud.srv"
  "srv/DetachPointCloud.srv"
  "srv/AttachCalibration.srv"
  "srv/AttachCalibrationStereo.srv"
  "srv/DetachCalibration.srv"
  "srv/CameraStatus.srv"
  "srv/SwitchFlatCorrugated.srv"
  "srv/CameraCtrl.srv"
  "srv/MultiCameraCtrl.srv"

  "srv/Trim.srv"
  "srv/TrimDepth.srv"
  "srv/TrimDist.srv"
  "srv/TrimFT.srv"
  "srv/TrimPitch.srv"

  "srv/SystemctlCall.srv"

  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs mavros_msgs
)

ament_package()

# Create the missing file needed for the pkg to be discoverable using source
add_custom_target(
 run ALL
 COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/read_xml.py ${CMAKE_CURRENT_SOURCE_DIR}/package.xml ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
 COMMENT "Generating source package"
)

install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
  DESTINATION share/colcon-core/packages)
  
set(CPACK_PACKAGE_NAME "blueatlasrobotics-${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "Blue Atlas Robotics")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Blue Atlas Robotics <contact@blueatlasrobotics.com>")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/Blue-Atlas-Robotics/fishbot")
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
SET(CPACK_GENERATOR "DEB")
SET(CPACK_SET_DESTDIR ON)
SET(CPACK_INSTALL_PREFIX "/opt/ros/$ENV{ROS_DISTRO}/install/${PROJECT_NAME}")
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/postinst") 
INCLUDE(CPack)
