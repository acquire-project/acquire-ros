cmake_minimum_required(VERSION 3.23)
project(acquire_zarr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(external)

externalproject_get_property(acquire-zarr-library INSTALL_DIR)
set(ACQUIRE_ZARR_INSTALL_DIR ${INSTALL_DIR})
add_library(zarr_writer_node SHARED src/zarr_writer_node.cpp)

target_compile_definitions(zarr_writer_node PRIVATE "COMPOSITION_BUILDING_DLL")

target_link_libraries(zarr_writer_node PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
add_dependencies(zarr_writer_node acquire-zarr-library)

target_include_directories(zarr_writer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ACQUIRE_ZARR_INSTALL_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(zarr_writer_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

rclcpp_components_register_node(zarr_writer_node
  PLUGIN "acquire_zarr::ImageZarrWriterNode"
  EXECUTABLE image_zarr_writer_node)

rclcpp_components_register_node(zarr_writer_node
  PLUGIN "acquire_zarr::Float32MultiArrayZarrWriterNode"
  EXECUTABLE float_volume_zarr_writer)

target_link_directories(zarr_writer_node PUBLIC $<BUILD_INTERFACE:${ACQUIRE_ZARR_INSTALL_DIR}/lib>)
target_link_libraries(zarr_writer_node PUBLIC acquire-zarr)

# Install the library and executables
ament_export_targets(export_zarr_writer_node)
install(TARGETS zarr_writer_node
        EXPORT export_zarr_writer_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install the external acquire-zarr library along with the shared libraries it
# links against (e.g. miniocpp), which the node dlopens at runtime.
install(DIRECTORY ${ACQUIRE_ZARR_INSTALL_DIR}/lib/
  DESTINATION lib
  FILES_MATCHING
  PATTERN "*${CMAKE_SHARED_LIBRARY_SUFFIX}*"
  PATTERN "cmake" EXCLUDE
)

# Install launch & config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # external/ holds the vendored acquire-zarr and vcpkg submodules: tens of
  # thousands of third-party files that are not ours to lint. Most linters skip
  # it via the external/AMENT_IGNORE marker; pep257 honours neither that nor
  # AMENT_LINT_AUTO_FILE_EXCLUDE, so its hook is replaced below.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  if(COMMAND ament_pep257)
    ament_pep257(launch)
  endif()
endif()

ament_package()
