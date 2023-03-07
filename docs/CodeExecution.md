# Code Execution

## Building

In RosDockerWorkspace git:(main):

```
catkin build 
```

To source the zsh file (you need to run this once per terminal session):

```
wssetup
```

whenever you add/remove header files, or if you see clang errors:

```
catkin clean
```

## Running the MuJoCo Simulation

```
rosrun hopper_mpc hopper_mpc mujoco start_stand stand 5000 --plot
```

## Running Hardware Control

### Starting the IMU Publisher
Before the hardware controller can be started, the IMU docker image must be started. 

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
wssetup
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
         - A leg movement test, should be paired with `fixed`
      - `rotorvel` and `rotorpos`
         - Reaction wheel movement tests, should be paired with `fixed` start

   - N_run
      - Specify the number of timesteps to run for
   - `--plot`
      - Enable plotting
   - `--skip_homing` 
      - Don't home leg positions (on hardware)
   - `--skip_kf`
      - Don't use the kalman filter