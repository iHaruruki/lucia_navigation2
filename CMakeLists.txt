################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(lucia_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

################################################################################

ament_python_install_package(${PROJECT_NAME})

# Pythonノードをインストール
install(PROGRAMS
  lucia_navigation2/waypoint_navi_loop.py
  lucia_navigation2/waypoint_navi.py
  lucia_navigation2/patrol.py
  lucia_navigation2/waypoint_skip_tatto.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
