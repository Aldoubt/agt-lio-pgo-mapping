cmake_minimum_required(VERSION 3.8)
project(agt_map_studio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(agt_pcd2grid_exporter REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

# Qt6 is preferred when available, but this workspace currently provides Qt5.
find_package(Qt6 QUIET COMPONENTS Widgets OpenGLWidgets)
if(Qt6_FOUND)
  set(QT_WIDGETS_TARGET Qt6::Widgets)
  set(QT_OPENGL_TARGET Qt6::OpenGLWidgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
  set(QT_WIDGETS_TARGET Qt5::Widgets)
  set(QT_OPENGL_TARGET Qt5::OpenGL)
endif()

add_executable(map_viewer
  src/main.cpp
  src/io/PCDLoader.cpp
  src/viewer/CameraController.cpp
  src/viewer/PointCloudViewer.cpp
  src/selection/SelectionBox.cpp
  src/selection/SelectionManager.cpp
  src/occupancy/GridMap.cpp
  src/occupancy/MapYamlLoader.cpp
  src/occupancy/OccupancyViewer.cpp
  src/occupancy/RefinementModel.cpp
  src/occupancy/commands/EraseRectangleCommand.cpp
  src/occupancy/commands/DrawObstacleCommand.cpp
  src/occupancy/commands/ForbiddenPolygonCommand.cpp
  src/ui/MainWindow.cpp
)

target_include_directories(map_viewer PRIVATE
  src
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(map_viewer PRIVATE ${PCL_DEFINITIONS})

ament_target_dependencies(map_viewer
  ament_index_cpp
)

target_link_libraries(map_viewer
  ${PCL_LIBRARIES}
  ${QT_WIDGETS_TARGET}
  ${QT_OPENGL_TARGET}
  yaml-cpp
  agt_pcd2grid_exporter::agt_pcd2grid_exporter
  OpenGL::GL
)

install(TARGETS map_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pcd_loader
    test/test_pcd_loader.cpp
    src/io/PCDLoader.cpp
  )
  target_include_directories(test_pcd_loader PRIVATE src ${PCL_INCLUDE_DIRS})
  target_compile_definitions(test_pcd_loader PRIVATE ${PCL_DEFINITIONS})
  ament_target_dependencies(test_pcd_loader ament_index_cpp)
  target_link_libraries(test_pcd_loader ${PCL_LIBRARIES})

  ament_add_gtest(test_selection_manager
    test/test_selection_manager.cpp
    src/io/PCDLoader.cpp
    src/selection/SelectionManager.cpp
  )
  target_include_directories(test_selection_manager PRIVATE src ${PCL_INCLUDE_DIRS})
  target_compile_definitions(test_selection_manager PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(test_selection_manager
    ${PCL_LIBRARIES}
    ${QT_WIDGETS_TARGET}
    yaml-cpp
  )

  ament_add_gtest(test_occupancy_viewer
    test/test_occupancy_viewer.cpp
    src/occupancy/GridMap.cpp
    src/occupancy/MapYamlLoader.cpp
    src/occupancy/RefinementModel.cpp
    src/occupancy/commands/EraseRectangleCommand.cpp
    src/occupancy/commands/DrawObstacleCommand.cpp
    src/occupancy/commands/ForbiddenPolygonCommand.cpp
  )
  target_include_directories(test_occupancy_viewer PRIVATE src)
  target_link_libraries(test_occupancy_viewer yaml-cpp)

  ament_add_gtest(test_occupancy_refinement
    test/test_occupancy_refinement.cpp
    src/occupancy/GridMap.cpp
    src/occupancy/MapYamlLoader.cpp
    src/occupancy/RefinementModel.cpp
    src/occupancy/commands/EraseRectangleCommand.cpp
    src/occupancy/commands/DrawObstacleCommand.cpp
    src/occupancy/commands/ForbiddenPolygonCommand.cpp
  )
  target_include_directories(test_occupancy_refinement PRIVATE src)
  target_link_libraries(test_occupancy_refinement yaml-cpp)
endif()

ament_package()
