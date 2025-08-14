cmake_minimum_required(VERSION 3.8)
project(cxd5602pwbimu_localizer_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(localizer_node src/localizer_node.cpp)
ament_target_dependencies(localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  std_srvs
)

target_link_libraries(localizer_node serial)

install(TARGETS
  localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()