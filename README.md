# Project Description

The goal of this project is to develop a Python program that allows a TurtleBot robot to autonomously navigate in a closed area and detect the presence of green ghosts.
The robot is capable of avoiding obstacles it detects through a laser scan. It recognizes objects using a camera mounted on the robot's head.


## Prerequisites

To test the project, make sure the following steps are completed:
Install ROS2 on your machine.
You can follow the official documentation here: 

    
    https://docs.ros.org/en/iron/Installation.html
    

Verify that Python 3 is installed on your machine and check the value of the ROS_Distro variable (ROS_DISTRO=iron).

Create a special package, which we can call ros_space, to hold the various GitHub repositories and libraries needed for the project.
You can use the following commands in your terminal:

        
        ros2 pkg create --build-type ament_cmake ros_space
        colcon list
        colcon build
        

Set the ROS_DOMAIN_ID to value 24 by running the following commands:

        
        export ROS_DOMAIN_ID=24
        echo "export ROS_DOMAIN_ID=24" >> ~/.bashrc
        

Add additional libraries (pkg-interfaces, realsense2):
You can use the following commands:
pkg-interfaces :

        cd ros_space
        git clone https://github.com/imt-mobisyst/pkg-interfaces.git
        colcon build --base-path pkg-interfaces
        source ./install/setup.bash
        
realsense2 :
        
        git clone https://github.com/Microsoft/vcpkg.git
        cd vcpkg
        ./bootstrap-vcpkg.sh
        ./vcpkg integrate install
        ./vcpkg install realsense2
        
    
Install Gazebo software by running the following commands in your terminal:

        
        cd ~/ros_space
        git clone https://github.com/imt-mobisyst/pkg-tsim
        colcon build
        source ./install/setup.bash
        



### Project Installation

To install the project, clone the A2-S4 GitHub repository using the following commands:

    cd ros-workspace
    git clone https://github.com/NABL-A/LARM-24.git
    
#### Usage

To start the simulation, use the following command, which will open a Gazebo window and launch the robot's movement simulation:
    
    ros2 launch grp_24 simulation_launch.yaml

to start the TurtleBot's navigation, use the following command:
   
    ros2 launch grp_24 tbot_launch.yaml