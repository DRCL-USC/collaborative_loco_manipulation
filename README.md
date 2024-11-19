# Adaptive Motion Planning for Safety-Critical Collaborative Loco-manipulation
This repository contains the code for the paper "[Hierarchical Adaptive Motion Planning with Nonlinear Model Predictive Control for Safety-Critical Collaborative Loco-Manipulation](https://arxiv.org/abs/2411.10699v1)." This is a hierarchical control system for object manipulation using a team of quadrupedal robots. A high-level NMPC planner generates collision-free paths, and the decentralized loco-manipulation controller ensures each robot maintains stable locomotion and manipulation based on the planner’s guidance.

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
``sudo apt install liburdfdom-dev libassimp-dev libglpk-dev libmpfr-dev``
* Optional: To plot planner data, install rqt-multiplot with `sudo apt-get install ros-noetic-rqt-multiplot`


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
git clone --depth 1 --recurse-submodules https://github.com/DRCL-USC/collaborative_loco_manipulation.git
```
Build simulation package for the planner as well as loco-manipulation controller:
```
cd ..
catkin build ocs2_object_manipulation_ros quadruped_sim 
```

# Usage
To facilitate running the motion planner stack and the decentralized loco-manipulation controller stack, we provide a Tmux script to manage the entire stack efficiently. To use this script, first install Tmux and Tmuxp:

```
sudo apt-get install tmux tmuxp
```

Next, navigate to the scripts directory and load the Tmux session:

```
cd src/collaborative_loco_manipulation/scripts
tmuxp load simulation.yaml
```

The Tmux session includes a window for the motion planner and two additional windows for decentralized loco-manipulation control for each robot. Detailed information about the loco-manipulation stack can be found [here](https://github.com/DRCL-USC/Loco_manipulation_control).

The planner stack starts with this launch file, which loads all the core nodes:

```
roslaunch ocs2_object_manipulation_ros manipulation_stack.launch gui:=false rviz:=true multiplot:=false record_data:=false
```

This launch file contains several arguments. Setting `gui` to true enables Gazebo simulations. `rviz` enables RViz data visualization. `multiplot` plots the planner variables online, and `record_data` stores rostopic data related to the planner and decentralized controller into a ROS bag file.

Once the stack is running, a new terminal will pop up, allowing you to enter the target position and orientation for the manipulated object. After specifying the target, the manipulation process will begin.

# Citation 
```
@misc{Sombolestan2024HierarchicalLoco-Manipulation,
    title = {{Hierarchical Adaptive Motion Planning with Nonlinear Model Predictive Control for Safety-Critical Collaborative Loco-Manipulation}},
    year = {2024},
    author = {Sombolestan, Mohsen and Nguyen, Quan},
    month = {11},
    url = {https://arxiv.org/abs/2411.10699v1},
    arxivId = {2411.10699}
}
```