cmake_minimum_required(VERSION 3.23)
project(acquire-zarr-external)

# Bootstrap vcpkg
set(VCPKG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg)
add_custom_target(vcpkg
        COMMAND ./bootstrap-vcpkg.sh -disableMetrics
        WORKING_DIRECTORY ${VCPKG_ROOT}
        COMMENT "Bootstrapping vcpkg"
)
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

# Include ExternalProject module
include(ExternalProject)

# Define acquire-zarr-driver as an ExternalProject
set(AZ_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-zarr-build)
set(AZ_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-zarr-install)
ExternalProject_Add(
        acquire-driver-zarr
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/acquire-driver-zarr
        BINARY_DIR ${AZ_BUILD_DIR}
        INSTALL_DIR ${AZ_INSTALL_DIR}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} -B ${AZ_BUILD_DIR} -DBUILD_SHARED_LIBS=ON -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake ${CMAKE_CURRENT_SOURCE_DIR}/acquire-driver-zarr
        INSTALL_COMMAND ${CMAKE_COMMAND} --install ${AZ_BUILD_DIR} --prefix ${AZ_INSTALL_DIR}
        COMMENT "Building acquire-zarr-driver"
        DEPENDS vcpkg
        EXCLUDE_FROM_ALL TRUE
        LOG_CONFIGURE TRUE
        LOG_BUILD TRUE
        LOG_INSTALL TRUE
        LOG_OUTPUT_ON_FAILURE TRUE
)
