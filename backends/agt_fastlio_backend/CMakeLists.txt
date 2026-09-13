cmake_minimum_required(VERSION 3.8)
project(agt_fastlio_backend)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agt_mapping_frontend_api REQUIRED)

add_library(${PROJECT_NAME}_relay src/frontend_relay.cpp)
target_include_directories(${PROJECT_NAME}_relay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_relay nav_msgs sensor_msgs)

add_executable(fastlio_backend_node src/fastlio_backend_node.cpp)
target_link_libraries(fastlio_backend_node ${PROJECT_NAME}_relay)
ament_target_dependencies(fastlio_backend_node rclcpp nav_msgs sensor_msgs agt_mapping_frontend_api)

install(TARGETS ${PROJECT_NAME}_relay fastlio_backend_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fastlio_frontend_relay test/test_fastlio_frontend_relay.cpp)
  target_link_libraries(test_fastlio_frontend_relay ${PROJECT_NAME}_relay)
  ament_target_dependencies(test_fastlio_frontend_relay nav_msgs sensor_msgs agt_mapping_frontend_api)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_relay)
ament_export_dependencies(nav_msgs sensor_msgs)
ament_package()
