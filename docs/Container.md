# Using the Docker Container
[RosDockerWorkspace](https://github.com/RoboticExplorationLab/RosDockerWorkspace) serves as the environment for REx Hopper. However, we do not recommend building it as a docker container on the robot's computer due to both a lack of space and driver incompatibilities.

If you are building this on a separate computer for simulation purposes, go ahead.

```
git clone git@github.com:RoboticExplorationLab/RosDockerWorkspace.git
cd RosDockerWorkspace
mkdir src
cd src
git clone git@github.com:RoboticExplorationLab/RExHopper.git
```

**WARNING: Only build on a remote computer for remote work (e.g. simulation).**

**DO NOT BUILD ON THE HARDWARE COMPUTER.**

To build the container in VScode:

   * Click on the green button in the lower left -> Reopen in Container

If you change the Dockerfile or devcontainer.json and want to rebuild the env:

   * Ctrl+shift+p -> rebuild container
