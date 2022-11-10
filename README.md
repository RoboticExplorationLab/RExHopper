# REx Hopper

The REx Hopper is a monopodal hopping robot with reaction wheels. This repository contains C++ code for control of the robot, both on hardware and in simulation.

## UP Xtreme Setup Instructions

The following are instructions for setting up the REx Hopper computer such that it is compatible with the RosDockerWorkspace environment.

1. Update the UEFI BIOS using these [instructions] (https://downloads.up-community.org/download/up-xtreme-uefi-bios-v1-9/).
   - Use `GO.nsh` rather than `GO_Entire.nsh`.

2. Update the [kernel](https://github.com/up-board/up-community/wiki/Ubuntu_20.04).

3. Setup user GPIO/SPI/I2C [Permissions](https://github.com/up-board/up-community/wiki/Ubuntu_20.04#enable-the-hat-functionality-from-userspace).

4. Install Docker. But do NOT build the RosDockerWorkspace container. It will take up too much filespace and is incompatible with the PCAN drivers.

## Repository Setup Instructions

[RosDockerWorkspace](https://github.com/RoboticExplorationLab/RosDockerWorkspace) serves as the environment for REx Hopper. However, we do not recommend building it as a docker container on the robot's computer due to both a lack of space and driver incompatibilities.

```
git clone git@github.com:RoboticExplorationLab/RosDockerWorkspace.git
cd RosDockerWorkspace
mkdir src
cd src
git clone git@github.com:RoboticExplorationLab/RExHopper.git
```

## Using the Container
Currently, the docker container is not usable for code execution. As previously stated, we do not recommend building the docker container on the robot's computer due to both a lack of storage space and driver incompatibilities. It can only be used to build on another computer for remote work.

In VScode:
Click on green button in lower left -> reopen in container
Whenever you change Dockerfile and want to rebuild the env ->  Ctrl+shift+p -> rebuild container

## Starting the IMU publisher

Before the code can be executed, the IMU docker image must be started. If you haven't already, pull the microstrain-inertial ROS Docker image for the [3DM AHRS](https://hub.docker.com/r/microstrain/ros-microstrain_inertial_driver).

```
sudo docker pull microstrain/ros-microstrain_inertial_driver:ros
```

Find the device id with 

```
dmesg | grep tty
```

In RosDockerWorkspace, assuming the device id is the default `/dev/ttyACM0`, run:

```
sudo docker run -it --rm --net=host --device=/dev/ttyACM0 -v "/home/hopper/Documents/git_workspace/RosDockerWorkspace/src/RExHopper/params.yml:/tmp/params.yml" microstrain/ros-microstrain_inertial_driver:ros params_file:=/tmp/params.yml
```
Otherwise, as an example:
```
sudo docker run -it --rm --net=host --device=/dev/ttyACM1 -v "/home/hopper/Documents/git_workspace/RosDockerWorkspace/src/RExHopper/params.yml:/tmp/params.yml" microstrain/ros-microstrain_inertial_driver:ros params_file:=/tmp/params.yml port:=/dev/ttyACM1
```

You can check what it's publishing with 
```
rostopic list
```

## Mocap Node
```
roslaunch hopper_mpc mocap.launch
```

## Code Execution

In RosDockerWorkspace git:(main):

```
catkin build 
wssetup
rosrun hopper_mpc hopper_mpc mpc 5000 mujoco
rosrun hopper_mpc hopper_mpc mpc 5000 raisim
rosrun hopper_mpc hopper_mpc mpc 5000 hardware
```

whenever you add/remove header files, or if you see clang errors:

```
catkin clean
```