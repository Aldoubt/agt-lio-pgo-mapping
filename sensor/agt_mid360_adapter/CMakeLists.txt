cmake_minimum_required(VERSION 3.8)
project(agt_mid360_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_library(${PROJECT_NAME}_conversion src/custommsg_to_pointcloud.cpp)
target_include_directories(${PROJECT_NAME}_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_conversion sensor_msgs livox_ros_driver2)

add_executable(mid360_adapter_node src/mid360_adapter_node.cpp)
target_link_libraries(mid360_adapter_node ${PROJECT_NAME}_conversion)
ament_target_dependencies(mid360_adapter_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS ${PROJECT_NAME}_conversion mid360_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custommsg_to_pointcloud test/test_custommsg_to_pointcloud.cpp)
  target_link_libraries(test_custommsg_to_pointcloud ${PROJECT_NAME}_conversion)
  ament_target_dependencies(test_custommsg_to_pointcloud sensor_msgs livox_ros_driver2)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_conversion)
ament_export_dependencies(sensor_msgs livox_ros_driver2)
ament_package()
