# RExHopper
(Draft)

git clone RosDockerWorkspace
cd RosDockerWorkspace
mkdir src
cd src
git clone RExHopper

In VScode:
Click on green button in lower left -> reopen in container
Whenever you change Dockerfile and want to rebuild the env ->  Ctrl+shift+p -> rebuild container

Make new terminal in VScode:
In RosDockerWorkspace git:(main):
catkin build 
wssetup
rosrun hopper_mpc hopper_mpc mpc 5000 mujoco
rosrun hopper_mpc hopper_mpc mpc 5000 raisim
rosrun hopper_mpc hopper_mpc mpc 5000 hardware

whenever you add/remove header files, or if you see clang errors:
catkin clean

Additional notes:
- Raisim needs to additionally be installed externally to the container in order to run raisimUnityOpengl. The docker container then communicates through TCP with Opengl
- PEAK PCAN is not compatible with docker containers and needs to be installed outside of the container.