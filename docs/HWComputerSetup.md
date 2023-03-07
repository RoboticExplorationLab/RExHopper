# UP Xtreme First Time Setup Instructions

*This is only for setting up a new computer to control the hardware! If you're using the computer that has already been set up, you can skip this section.*

The following are instructions for setting up the REx Hopper computer such that it is compatible with the RosDockerWorkspace environment.

1. Update the UEFI BIOS using [these instructions](https://downloads.up-community.org/download/up-xtreme-uefi-bios-v1-9/).
   - Use `GO.nsh` rather than `GO_Entire.nsh`.

2. Update the [kernel](https://github.com/up-board/up-community/wiki/Ubuntu_20.04).

3. Setup user GPIO/SPI/I2C [Permissions](https://github.com/up-board/up-community/wiki/Ubuntu_20.04#enable-the-hat-functionality-from-userspace).

4. Install Docker. But do NOT build the RosDockerWorkspace container. It will take up too much filespace and is incompatible with the PCAN drivers.

5. Pull the microstrain-inertial ROS Docker image for the [3DM AHRS](https://hub.docker.com/r/microstrain/ros-microstrain_inertial_driver).

   ```
   sudo docker pull microstrain/ros-microstrain_inertial_driver:ros
   ```
6. See the [Environment Setup Doc](docs/EnvSetup.md).