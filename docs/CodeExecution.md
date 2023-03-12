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


# Debugging

Create a file named `launch.json` inside of the `.vscode` directory in RosDockerWorkspace. Copy + paste the following code into it. This is the  debugging config file.

```
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "MIMode": "gdb",
            "miDebuggerPath": "gdb",
            "program": "${workspaceFolder}/devel/lib/hopper_mpc/hopper_mpc",
            "args": [
                "mujoco",
                "start_stand",
                "stand",
                "5000",
                "--plot"
            ],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

You will need gdb. If it is not already installed:

```
sudo apt update
sudo apt install gdb
```

Build the workspace with debugging enabled (make sure to clean first)...

```
catkin clean
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Then, simply go to Run & Debug (CTRL+SHIFT+D) and hit the Start Debugging button (or F5).

### Debugging with MuJoCo

To do this, you will have to uninstall MuJoCo and then rebuild it with debugging enabled.

```
cd ~/mujoco/build_dir
sudo make uninstall
sudo cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mujoco -DCMAKE_BUILD_TYPE=Debug
sudo cmake --build . && sudo make install
```
