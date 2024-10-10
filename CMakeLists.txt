cmake_minimum_required(VERSION 3.5)
project(beefast_single_laser)

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
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(beefast_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
)

# Get source files
file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB LDLIDAR_SDK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/*.cpp)

# Verify that the static library exists
set(LDLIDAR_SDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/libldlidar_driver_ssl.a)
if(NOT EXISTS ${LDLIDAR_SDK_LIB})
  message(FATAL_ERROR "Static library not found: ${LDLIDAR_SDK_LIB}")
endif()

# Generate binary file and link
add_executable(single_laser 
  src/main.cpp
  src/life_single_laser.cpp 
  ${LDLIDAR_SDK_SRC}
)
add_executable(pub_distance
  src/ranging_along_edges.cpp
)

add_executable(cliff_detection
  src/cliff_detection.cpp
)

ament_target_dependencies(pub_distance 
  rclcpp 
  sensor_msgs
  nav2_util
  beefast_interfaces
)

ament_target_dependencies(cliff_detection
  rclcpp 
  sensor_msgs
  nav2_util
  beefast_interfaces
)

# Use the executable target name instead of project name
ament_target_dependencies(single_laser 
  rclcpp 
  sensor_msgs
  nav2_util
)

# Link the library and other dependencies
target_link_libraries(single_laser 
  ${LDLIDAR_SDK_LIB}
  pthread 
)

# Install targets
install(TARGETS single_laser
  DESTINATION lib/${PROJECT_NAME}
)
# ��װĿ��
install(TARGETS pub_distance
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS cliff_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and rviz2 files
install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
