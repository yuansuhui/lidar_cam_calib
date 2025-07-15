cmake_minimum_required(VERSION 3.10)
project(foxglove_lidar_calib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calibrator_node src/calibrator_node.cpp)
ament_target_dependencies(calibrator_node rclcpp sensor_msgs cv_bridge geometry_msgs)

target_link_libraries(calibrator_node ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS calibrator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME})

ament_package()
