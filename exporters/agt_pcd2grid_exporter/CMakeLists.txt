cmake_minimum_required(VERSION 3.8)
project(agt_pcd2grid_exporter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/PCDProjector.cpp
  src/OccupancyGridWriter.cpp
  src/ParameterLoader.cpp
  src/TemporalPersistenceFilter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} yaml-cpp)

add_executable(pcd2grid_exporter src/main.cpp)
target_link_libraries(pcd2grid_exporter ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} pcd2grid_exporter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pcd2grid
    test/test_pcd2grid.cpp
  )
  target_include_directories(test_pcd2grid PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd2grid ${PROJECT_NAME} ${PCL_LIBRARIES} yaml-cpp)
endif()

ament_package()
