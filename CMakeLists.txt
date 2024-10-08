cmake_minimum_required(VERSION 3.5)

# Extract package name and version from package.xml using macros from `ros_industrial_cmake_boilerplate` (RICB)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# Find `noether_gui`
find_package(noether_gui REQUIRED)

# Add a library for our custom tool path planning components
add_library(${PROJECT_NAME} SHARED
  # Exercise 2a
  src/exercise_2a/cylinder_segmentation_mesh_modifier.cpp
  src/exercise_2a/cylinder_segmentation_mesh_modifier_widget.cpp
  # Exercise 2b
  src/exercise_2b/no_op_tool_path_planner.cpp
  src/exercise_2b/no_op_tool_path_planner_widget.cpp
  # Exercise 2c
  src/exercise_2c/camera_standoff_tool_path_modifier_widget.cpp
)
# Link against headers from this repository
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# Link against dependency targets
target_link_libraries(${PROJECT_NAME} PUBLIC
  noether::noether_gui
)
# Set a specific cxx version using RICB macro
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)

# Plugin library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Package configuration via RICB
configure_package(
  NAMESPACE noether
  DEPENDENCIES
    noether_gui
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
)
