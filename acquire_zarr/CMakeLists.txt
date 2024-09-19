cmake_minimum_required(VERSION 3.8)
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

add_subdirectory(external)
#option(ACQUIRE_ZARR_INSTALL_DIR "Install Location of the Acquire Zarr library" "${CMAKE_CURRENT_SOURCE_DIR}/acquire_driver_zarr/install")
ExternalProject_Get_Property(acquire-driver-zarr INSTALL_DIR)
set(ACQUIRE_ZARR_INSTALL_DIR ${INSTALL_DIR})
add_library(zarr_writer_node SHARED src/zarr_writer_node.cpp)

target_compile_definitions(zarr_writer_node PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(zarr_writer_node "rclcpp" "rclcpp_components" "sensor_msgs")
add_dependencies(zarr_writer_node acquire-driver-zarr)

target_include_directories(zarr_writer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ACQUIRE_ZARR_INSTALL_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

target_compile_features(zarr_writer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rclcpp_components_register_node(zarr_writer_node 
  PLUGIN "acquire_zarr::ZarrWriterNode" 
  EXECUTABLE zarr_writer_node_exe)

target_link_directories(zarr_writer_node PUBLIC $<BUILD_INTERFACE:${ACQUIRE_ZARR_INSTALL_DIR}/lib>)
target_link_libraries(zarr_writer_node acquire-zarr) 

# Install the library and executables
ament_export_targets(export_zarr_writer_node)
install(TARGETS zarr_writer_node
        EXPORT export_zarr_writer_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install the external acquire-zarr library
install(FILES ${ACQUIRE_ZARR_INSTALL_DIR}/lib/libacquire-zarr${CMAKE_SHARED_LIBRARY_SUFFIX} DESTINATION lib)

# Install launch & config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
