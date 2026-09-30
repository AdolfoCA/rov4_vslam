cmake_minimum_required(VERSION 3.16)
project(aqua_slam)

# AQUA-SLAM (Xu, Zhang, Wang, T-RO 2025), ported from ROS 1 catkin to ROS 2 ament.
# Upstream: https://github.com/SenseRoboticsLab/AQUA-SLAM - see UPSTREAM.md.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Upstream builds with warnings off; the research code is noisy and the port keeps it as is.
add_compile_options(-w)
add_definitions(-DCOMPILEDWITHC11 -DBOOST_LOG_DYN_LINK)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(bluerov2_msgs REQUIRED)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d calib3d highgui video ximgproc)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(octomap REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS log log_setup serialization system filesystem iostreams thread)

# Vendored ORB-SLAM3 dependencies (modified by the AQUA-SLAM authors; not the system g2o).
add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

set(ROS_DEPS
  rclcpp builtin_interfaces std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs
  visualization_msgs tf2_ros cv_bridge image_transport pcl_conversions octomap_msgs)

set(CORE_INCLUDES
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS})

# SLAM core. Split in two as upstream does: the optimisation units are large.
add_library(${PROJECT_NAME}_core SHARED
  src/ros1_compat.cpp
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/DvlGyroOptimizer.cpp
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/MLPnPsolver.cpp
  src/TwoViewReconstruction.cc
  src/RosHandling.cpp
  src/DenseMapper.cpp
  src/LKTracker.cpp
  src/FrameKLT.cpp
  src/Integrator.cpp)

# Memory: every unit that includes the SLAM headers (rclcpp, PCL, g2o, Eigen, Boost
# serialization) costs cc1plus ~3.3 GB before any code. The g2o-heavy units add more,
# so upstream's 14k-line Optimizer.cc is split into five parts (UPSTREAM.md), these
# units use -O2 instead of -O3, and GCC collects garbage eagerly. Peak per compiler
# process is then ~4 GB: build with ONE job on a 16 GB machine (see the Dockerfile).
set(LOW_MEMORY_COMPILE "--param=ggc-min-expand=20;--param=ggc-min-heapsize=65536")
set(OPTIMIZER_PARTS
  src/Optimizer_part1.cc src/Optimizer_part2.cc src/Optimizer_part3.cc
  src/Optimizer_part4.cc src/Optimizer_part5.cc)
set_source_files_properties(${OPTIMIZER_PARTS} src/G2oTypes.cc src/DvlGyroOptimizer.cpp
  PROPERTIES COMPILE_OPTIONS "-O2;${LOW_MEMORY_COMPILE}")

add_library(${PROJECT_NAME}_optimization SHARED
  ${OPTIMIZER_PARTS}
  src/ImuTypes.cc
  src/DVLGroPreIntegration.cpp
  src/G2oTypes.cc
  src/OptimizableTypes.cpp)

foreach(lib ${PROJECT_NAME}_core ${PROJECT_NAME}_optimization)
  target_include_directories(${lib} PUBLIC ${CORE_INCLUDES})
  target_link_libraries(${lib}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    Sophus::Sophus
    fmt::fmt
    OpenSSL::Crypto
    Boost::log Boost::log_setup Boost::serialization Boost::system Boost::filesystem Boost::iostreams Boost::thread
    DBoW2
    g2o)
  ament_target_dependencies(${lib} ${ROS_DEPS})
endforeach()
# The two halves call into each other (upstream links them the same way).
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_optimization)

add_executable(aqua_slam_node src/aqua_slam_node.cpp)
target_link_libraries(aqua_slam_node ${PROJECT_NAME}_core ${PROJECT_NAME}_optimization)
ament_target_dependencies(aqua_slam_node ${ROS_DEPS} bluerov2_msgs)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_optimization g2o DBoW2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(TARGETS aqua_slam_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
