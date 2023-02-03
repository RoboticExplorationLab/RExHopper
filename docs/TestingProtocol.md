# Testing Setup

1. Make sure the 48V power XT60 connector is disconnected.
2. Make sure the computer power jack is disconnected.
3. Make sure the 48V power switch is in the "off" position.
4. Make sure the battery module power switch (A.K.A. E-stop) is in the "off" position. The key should not be inserted.
5. Use the battery voltage checker to check that all of the battery voltages (both individual cells and each pack) are within 0.2 V of each other, and within the acceptable range. 
5. Hook the robot up to the harness and engage it with the jig release mechanism.
    - Use the bubble level to ensure that the robot is held by the harness and jig such that it is perfectly level.
6. You may now attach the computer power and battery power connectors.
7. Turn the computer on. Check that it is in working order.
    1. Hit the computer power button and wait for it to boot up.
    2. **From now on, while the computer is on, ALWAYS ground yourself before making contact with the robot--even if you only stepped away for a few seconds. Static charge builds up fast on carpet, and the computer is highly vulnerable to static shock. Any sparking will cause it to reboot.**
    2. Use Windows Remote Desktop to remote into hopper-UP-WHL01, or use the HDMI port and a wireless mouse+keyboard.
8. Insert the E-stop key and turn it to the "on" position.
9. Insert the power distribution key and turn it to the "on" position. The board LED should light up, as well as the ODrives.
10. You may now run the code.

# Running the code
1. Open up VS Code on your laptop. Make sure the Remote-SSH plugins are installed.
2. `ctrl+shift+p -> remote_ssh: Add New SSH Host`
3. Enter the host name and password, and .ssh. After doing this once, you can skip steps 2 and 3.
4. `ctrl+shift+p -> remote_ssh: Connect to SSH Host`
5. Type in the password.
6. File->Open Folder->Documents/git_workspace/RosDockerWorkspace
7. Start the AHRS node.
    ```
    sudo docker run -it --rm --net=host --device=/dev/ttyACM0 -v "/home/hopper/Documents/git_workspace/RosDockerWorkspace/src/RExHopper/params.yml:/tmp/params.yml" microstrain/ros-microstrain_inertial_driver:ros params_file:=/tmp/params.yml
    ```
8. In a separate terminal, start the mocap node (if you are doing mocap).
    ```
    wssetup
    roslaunch mocap_optitrack mocap.launch
    ```
    Note: You shouldn't need to start the ROS nodes more than once per testing session. As long as the computer isn't turned off.

9. In another separate terminal, run the control code.
    ```
    wssetup
    rosrun hopper_mpc hopper_mpc hardware start_stand stand 5000 --plot
    ```
Note: Movement during the startup and homing sequence may cause the jig to release early. Watch for this and reset the robot if this happens, before pressing any key to continue. Use the bubble level to ensure the robot is still well-oriented.

# During Testing
- If something goes wrong, use the E-stop to kill power.
- After hitting the E-stop, you will need to turn the power distribution board off as well. The circuit for power to the ODrives should always be completed by the power distribution switch, NOT the E-stop. 
    - This is because the power distribution board has pre-charging and arc protection for its switch. The E-stop switch does not have these features.
    - When ready to test again, always turn the E-stop on first and then the power distribution.

# Deactivation Protocol
1. Insert the power distribution key and turn it to the "off" position. The board LED and ODrive LEDs should go out.
2. Turn the E-stop to the "off" position and pull the key out. Place the cap back on.
3. Disconnect the 48V power connector(s).
4. Disconnect the computer power jack.
5. Take the robot down from the harness. Do NOT do this until you are sure that all power connectors have been removed.


