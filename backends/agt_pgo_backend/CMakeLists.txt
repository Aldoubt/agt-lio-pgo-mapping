cmake_minimum_required(VERSION 3.8)
project(agt_pgo_backend)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(agt_mapping_backend_api REQUIRED)
find_package(interface REQUIRED)
find_path(AGT_MAPPING_BACKEND_API_INCLUDE_DIR agt_mapping_backend_api/backend_topics.hpp PATH_SUFFIXES include)

add_library(${PROJECT_NAME}_bridge src/pgo_bridge.cpp)
target_include_directories(${PROJECT_NAME}_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_bridge geometry_msgs nav_msgs std_msgs)

add_executable(pgo_backend_node src/pgo_backend_node.cpp)
target_link_libraries(pgo_backend_node ${PROJECT_NAME}_bridge)
ament_target_dependencies(pgo_backend_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs agt_mapping_backend_api interface)
target_link_libraries(pgo_backend_node agt_mapping_backend_api::agt_mapping_backend_api__rosidl_typesupport_cpp)
target_include_directories(pgo_backend_node PRIVATE ${AGT_MAPPING_BACKEND_API_INCLUDE_DIR})

install(TARGETS ${PROJECT_NAME}_bridge pgo_backend_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pgo_backend_bridge test/test_pgo_backend_bridge.cpp)
  target_link_libraries(test_pgo_backend_bridge ${PROJECT_NAME}_bridge)
  ament_target_dependencies(test_pgo_backend_bridge geometry_msgs nav_msgs std_msgs agt_mapping_backend_api)
  target_link_libraries(test_pgo_backend_bridge agt_mapping_backend_api::agt_mapping_backend_api__rosidl_typesupport_cpp)
  target_include_directories(test_pgo_backend_bridge PRIVATE ${AGT_MAPPING_BACKEND_API_INCLUDE_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_bridge)
ament_export_dependencies(geometry_msgs nav_msgs std_msgs)
ament_package()
