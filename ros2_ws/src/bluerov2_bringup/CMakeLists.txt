cmake_minimum_required(VERSION 3.8)
project(bluerov2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_streams.py
  scripts/multicam.py
  scripts/preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
