# Adaptive Motion Planning for Safety-Critical Collaborative Loco-manipulation
This repository contains the code for the paper "Hierarchical Adaptive Motion Planning with Nonlinear Model Predictive Control for Safety-Critical Collaborative Loco-Manipulation". This is a hierarchical control system for object manipulation using a team of quadrupedal robots. A high-level NMPC planner generates collision-free paths and the decentralized loco-manipulation controller then ensures each robot maintains stable locomotion and manipulation based on the planner’s guidance.

## Video Demonstration
[![Video Title](https://img.youtube.com/vi/cU_qevkW86I/0.jpg)](https://www.youtube.com/watch?v=cU_qevkW86I)

# Installation
## Prerequisites

The library is written in C++11, and it is tested under Ubuntu 20.04 with library versions as 
provided in the package sources.

## Dependencies

* C++ compiler with C++11 support
* Eigen (v3.3)
* Boost C++ (v1.71)
* ROS Noetic
* LCM ``sudo apt-get install liblcm-dev``
* catkin ``sudo apt-get install catkin``
* pybind11_catkin, ROS package, installable via ``sudo apt install ros-noetic-pybind11-catkin``
* catkin-pkg package for python3. Install with ``sudo apt install python3-catkin-tools``
* Some other packages: 
``sudo apt install liburdfdom-dev libassimp-dev libglpk-dev libmpfr-dev ``


## Build the library

Create a new catkin workspace:

```
# Create the directories
# Do not forget to change <...> parts
mkdir -p <directory_to_ws>/<catkin_ws_name>/src
cd <directory_to_ws>/<catkin_ws_name>/

# Initialize the catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Clone the code:

```
# Navigate to the directory of src
cd <directory_to_ws>/<catkin_ws_name>/src
git clone --recurse-submodules 
```
Build simulation package for planner as well as loco-manipulation controller:
```
cd ..
catkin build ocs2_object_manipulation_ros quadruped_sim 
```

# Usage

# Citation 
