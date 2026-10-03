cmake_minimum_required(VERSION 3.8)
project(agt_mapping_frontend_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(agt_mapping_frontend_api REQUIRED)

add_library(${PROJECT_NAME}_normalizer src/frontend_normalizer.cpp)
target_include_directories(${PROJECT_NAME}_normalizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_normalizer
  nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_sensor_msgs)

add_executable(mapping_frontend_adapter_node src/frontend_adapter_node.cpp)
target_link_libraries(mapping_frontend_adapter_node ${PROJECT_NAME}_normalizer)
ament_target_dependencies(mapping_frontend_adapter_node rclcpp nav_msgs sensor_msgs
  geometry_msgs diagnostic_msgs message_filters livox_ros_driver2 agt_mapping_frontend_api)

install(TARGETS ${PROJECT_NAME}_normalizer mapping_frontend_adapter_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frontend_normalizer test/test_frontend_normalizer.cpp)
  target_link_libraries(test_frontend_normalizer ${PROJECT_NAME}_normalizer)
  ament_target_dependencies(test_frontend_normalizer nav_msgs sensor_msgs geometry_msgs
    tf2 tf2_geometry_msgs tf2_sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_normalizer)
ament_export_dependencies(nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_sensor_msgs)
ament_package()
