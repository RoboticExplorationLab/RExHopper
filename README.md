# REx Hopper

The REx Hopper is a monopodal hopping robot with reaction wheels. This repository contains C++ code for control of the robot, both on hardware and in simulation.

## UP Xtreme Setup Instructions

The following are instructions for setting up the REx Hopper computer such that it is compatible with the RosDockerWorkspace environment.

1. Update the UEFI BIOS using [these instructions](https://downloads.up-community.org/download/up-xtreme-uefi-bios-v1-9/).
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


<!-- ## First Time Setup of the 3DMCX5  (This is wrong)
1. Install [SensorConnect](https://www.microstrain.com/software/sensorconnect) on a Windows computer. Connect the 3DMCX5 by USB.
2. In SensorConnect, set the sensor ranges:
   - Accelerometer: 20g
   - Gyroscope: 500 deg/s
3. Set UART Baud Rate to 921600.
4. In Configuration > Mounting, set the following transformation in Euler Angles:
   [-0.5236, 0, 1.571]  (-30, 0, 90 deg)  <- cv7 actually this needs to  be set in the params.yml...
   [0, 0.5236, -1.571]  <- cx5
5. Save to startup configuration and disconnect. Connect to the Hopper computer via USB. -->


## Code Execution Setup
In RosDockerWorkspace git:(main):

```
catkin build 
wssetup
```

whenever you add/remove header files, or if you see clang errors:

```
catkin clean
```

### Running the MuJoCo Simulation

```
rosrun hopper_mpc hopper_mpc mujoco start_stand stand 5000 --plot
```

## Running Hardware Control

### Starting the IMU Publisher
Before the hardware controller can be started, the IMU docker image must be started. If you haven't already, pull the microstrain-inertial ROS Docker image for the [3DM AHRS](https://hub.docker.com/r/microstrain/ros-microstrain_inertial_driver).

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

### Mocap Node
In a separate terminal:
```
wssetup
roslaunch mocap_optitrack mocap.launch
```
### Hardware Control Initialization
```
rosrun hopper_mpc hopper_mpc hardware start_stand stand 5000 --plot
```

## Argparse Arguments Explained

   - bridge
      - `hardware`
         - Control the hardware
      - `mujoco`  
         - Run MuJoCo sim
   
   - start
      - `start_stand`
         - Start standing
      - `start_sit`
         - Start from a sitting position
      - `fixed`
         - The robot base is fixed to a jig

   - ctrl
      - `mpc`
         - Convex MPC (WIP)
      - `raibert`
         - Raibert hopping
      - `stand`
         - Balance while standing
      - `idle`
         - Do nothing
      - `circle`
         - A leg movement test, should be paired with `--fixed`
      - `rotorspeed`
         - A reaction wheel movement test, should be paired with `--fixed`

   - N_run
      - Specify the number of timesteps to run for
   - `--plot`
      - Enable plotting
   - `--skip_homing` 
      - Don't home leg positions (on hardware)
   - `--skip_kf`
      - Don't use the kalman filter

## ODrive Setup

See the [ODrive Setup doc](ODriveSetup.md).

## Creating New MJCFs

See the [Generating MJCFs doc](GeneratingMJCFs.md).