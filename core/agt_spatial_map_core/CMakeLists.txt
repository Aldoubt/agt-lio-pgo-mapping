cmake_minimum_required(VERSION 3.8)
project(agt_spatial_map_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/spatial_evidence.cpp
  src/spatial_evidence_builder.cpp
  src/spatial_export.cpp
  src/spatial_review_source.cpp
  src/geometry_evidence.cpp
  src/geometry_evidence_estimator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
  PRIVATE yaml-cpp OpenSSL::Crypto)

add_executable(agt_spatial_map_export src/spatial_map_export_main.cpp)
target_link_libraries(agt_spatial_map_export PRIVATE ${PROJECT_NAME})
install(TARGETS agt_spatial_map_export RUNTIME DESTINATION lib/${PROJECT_NAME})

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
  ament_add_gtest(test_spatial_export test/test_spatial_export.cpp)
  target_link_libraries(test_spatial_export ${PROJECT_NAME} yaml-cpp)
  ament_add_gtest(test_spatial_review test/test_spatial_review.cpp)
  target_link_libraries(test_spatial_review ${PROJECT_NAME} yaml-cpp)

  # Compile the actual unchanged legacy implementation into a regression-only
  # executable: no new dependency or behavior change in the old exporter.
  set(LEGACY_PCD2GRID_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../exporters/agt_pcd2grid_exporter")
  ament_add_gtest(test_temporal_filter_compat
    test/test_temporal_filter_compat.cpp
    "${LEGACY_PCD2GRID_ROOT}/src/TemporalPersistenceFilter.cpp")
  target_include_directories(test_temporal_filter_compat PRIVATE
    "${LEGACY_PCD2GRID_ROOT}/include")
  target_link_libraries(test_temporal_filter_compat ${PROJECT_NAME})
endif()

ament_package()
