cmake_minimum_required(VERSION 3.5)

# Extract package name and version from package.xml using macros from `ros_industrial_cmake_boilerplate` (RICB)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# Find `noether_tpp`, `noether_gui`, and Qt5 packages
find_package(noether_tpp REQUIRED)
find_package(noether_gui REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Package configuration via RICB
configure_package()
