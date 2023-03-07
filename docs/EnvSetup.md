# Setting Up the C++ Build Environment

This has been tested in wsl2 Ubuntu 20.04 and base Ubuntu 20.04.

1. Update your packages.
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt-get install software-properties-common
    ```

2. Install ROS. http://wiki.ros.org/ROS/Installation/TwoLineInstall/

    ```
    wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
    ```
    When prompted, choose option 2.

<!-- sudo apt install python3-rosdep
sudo apt install ros-noetic-rosbash
sudo apt-get install ros-noetic-rviz -->

3. Install the following prereqs.
    ```
    sudo apt -y install \
        curl \
        git \
        zsh \
        autojump \
        python3-catkin-tools \
        clang-format \
        libpopt-dev \
        libglfw3 \
        libglfw3-dev
    ```

4. Setup clang tools.
    ```
    git clone https://github.com/Huoleit/cmake_clang_tools.git /tmp/clang_tool
    cd /tmp/clang_tool && mkdir build && cd build
    sudo cmake .. && make 
    sudo make install
    ```

5. Follow this link and check the driver version:  https://www.peak-system.com/quick/PCAN-Linux-Driver
    
    e.g. peak-linux-driver-8.15.2.tar.gz

    And follow this link and check the API version: https://www.peak-system.com/quick/BasicLinux

    e.g. PCAN-Basic_Linux-4.6.2.tar.gz

    Record the version numbers.
    
    ```
    export PCAN_DRIVER_VERSION=8.15.2
    export PCAN_BASIC_API_VERSION=4.6.2
    export PCAN_BASIC_API_VERSION_ID=.36
    ```

6. Install the PCAN drivers and API.
    ```
    curl https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-${PCAN_DRIVER_VERSION}.tar.gz | tar -xz  -C /tmp/

    curl https://www.peak-system.com/produktcd/Develop/PC%20interfaces/Linux/PCAN-Basic_API_for_Linux/PCAN-Basic_Linux-${PCAN_BASIC_API_VERSION}.tar.gz | tar -xz  -C /tmp/

    export PCAN_DRIVER_DIR=/tmp/peak-linux-driver-${PCAN_DRIVER_VERSION}
    export ENV PCAN_BASIC_API_DIR=/tmp/PCAN-Basic_Linux-${PCAN_BASIC_API_VERSION}${PCAN_BASIC_API_VERSION_ID}

    cd /tmp/peak-linux-driver-8.15.2 && sudo make -C lib && sudo make -C lib install && sudo make -C test && sudo make -C test install

    sudo cp $PCAN_DRIVER_DIR/driver/pcan.h $PCAN_DRIVER_DIR/driver/pcanfd.h /usr/include/
    sudo chmod 644 /usr/include/pcan.h /usr/include/pcanfd.h

    sudo cp $PCAN_DRIVER_DIR/driver/lspcan /usr/local/bin/
    sudo chmod 755 /usr/local/bin/lspcan

    cd ${PCAN_BASIC_API_DIR}/libpcanbasic/pcanbasic && sudo make && sudo make install
    ```

7. Install MuJoCo.
    ```
    cd ~ && git clone https://github.com/deepmind/mujoco.git \
        && cd ~/mujoco && mkdir build_dir && cd build_dir \
        && sudo cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mujoco \
        && sudo cmake --build . && sudo make install
    ```

8. Install MRAA.
    ```
    sudo add-apt-repository ppa:up-division/mraa \
        && sudo apt-get update \
        && sudo apt-get install mraa-tools mraa-examples libmraa2 libmraa-dev libupm-dev libupm2 upm-examples
    ```

9. Set up the RosDockerWorkspace and RExHopper repositories.
    ```
    cd ~
    git clone https://github.com/RoboticExplorationLab/RosDockerWorkspace.git
    cd RosDockerWorkspace/src
    git checkout feature/bridge
    git clone https://github.com/RoboticExplorationLab/RExHopper.git
    cd RExHopper
    git checkout devel
    cd ..
    cd ..
    ```

10. 
    ```
    cp -r .devcontainer/scripts/ /tmp/scripts/
    chown -R $USER:$USER /tmp/scripts
    chsh
    ```
    When prompted, enter:
    ```
    /bin/zsh
    ```

11. Run the postCreate script and switch to zsh.
    ```
    sh /tmp/scripts/postCreate.sh
    zsh
    ```

12. If using wsl2, make sure the display variable is correct. If not using wsl2, skip this step.
    ```
    export DISPLAY=:0  # if using wsl2 ONLY
    ```

12. Test. This should work now.
    ```
    catkin clean
    catkin build
    wssetup
    rosrun hopper_mpc hopper_mpc mujoco start_stand stand 5000 --plot
    ```
