cmake_minimum_required(VERSION 3.8)
project(agt_spatial_map_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

add_library(${PROJECT_NAME} SHARED src/spatial_evidence.cpp src/spatial_evidence_builder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_spatial_evidence test/test_spatial_evidence.cpp)
  target_link_libraries(test_spatial_evidence ${PROJECT_NAME})
  ament_add_gtest(test_spatial_evidence_builder test/test_spatial_evidence_builder.cpp)
  target_link_libraries(test_spatial_evidence_builder ${PROJECT_NAME})
endif()

ament_package()
