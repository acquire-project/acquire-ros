# acquire_zarr
This package wraps the acquire-driver-zarr standalone library into a ROS node that subscribes to an image topic and streams them to a Zarr file.  

'As of this writing, this is still a work-in-progess, and many features which are supported by the library, are not yet supported here'''

## Build instructions

For now, since the build of the [acquire-driver-zarr](https://github.com/acquire-project/acquire-driver-zarr) library is complicated by its dependence on vcpkg, its build is kept independent of the ROS2 build. Please clone that repository separately, and build and install acquire-driver-zarr as directed by its [README](https://github.com/acquire-project/acquire-driver-zarr/blob/main/README.md) file with one exception: tell it to build the library as a shared/dynamic library using by adding `-DBUILD_SHARED_LIBS=ON`.   Install these on your system somewhere ROS2 & colcon can find them (like `/usr/local`) by the path as a --prefix in the installtion command:  `sudo cmake --install build --prefix /usr/local` (you will need root access to install here, hence the `sudo`).