# acquire_ros

This repository contains ROS2 wrappers around the acquire drivers, allowing seamless integration of acquire camera and storage drivers into ROS2-based projects.

## Build instructions
Of course, you must have [ROS2 installed](https://docs.ros.org/en/rolling/Installation.html), and [a workspace created](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#create-a-workspace) into which you will clone this repo.  Build using colcon as directed in the workspace link.

This repository maintains its dependencies through git submodules.  It is therefore important to clone this repository recursively (`git clone --recurse-submodules ...`) and/or to update them in the source tree (`git submodule update --init`).

Package-specific instructions are in the README files of the packages. 