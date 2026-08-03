cmake_minimum_required(VERSION 3.8)
project(tunnel_anomaly_detection)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration segmentation)
add_definitions(${PCL_DEFINITIONS})

add_executable(tunnel_anomaly_detector src/tunnel_anomaly_detector.cpp)
ament_target_dependencies(
  tunnel_anomaly_detector
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions)

target_include_directories(tunnel_anomaly_detector PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(tunnel_anomaly_detector ${PCL_LIBRARIES})
target_compile_features(tunnel_anomaly_detector PUBLIC cxx_std_17)

install(TARGETS tunnel_anomaly_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
