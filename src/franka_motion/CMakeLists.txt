cmake_minimum_required(VERSION 3.5.1)
project(franka_motion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(franka_interfaces REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
message("libfranka version: ${Franka_VERSION}")
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(RobotNode src/RobotNode.cpp src/RobotServer.cpp 
src/Robot.cpp src/Pose.cpp src/geometry_math.cpp src/PoseGenerator.cpp src/Payload.cpp src/StopCondition.cpp src/SpeedProfile.cpp src/SCurve.cpp 
src/examples_common.cpp)
target_include_directories(RobotNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(RobotNode
  rclcpp
  std_srvs
  Eigen3
  Franka
  franka_interfaces
)

install(TARGETS RobotNode
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
