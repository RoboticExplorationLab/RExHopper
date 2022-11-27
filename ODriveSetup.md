# ODrive Setup

## Settings
Overvoltage Trip Level: If using a power supply, use the max voltage of the power supply + 1.
```
odrv0.config.dc_bus_overvoltage_trip_level = 25  
```
Or if using the batteries, use the number of cells * 4.25.
```
odrv0.config.dc_bus_overvoltage_trip_level = 51
```
Undervoltage Trip Level: Use the number of cells * 3.3.
```
odrv0.config.dc_bus_undervoltage_trip_level = 39.6
```

Max Positive Current: Use the C rating on your batteries.
```
odrv0.config.dc_max_positive_current = 600  # 120 * 5 
```
Max Negative Current: -1 unless you have regen braking.
```
odrv0.config.dc_max_negative_current = -1
```

Use the onboard encoder:
```
odrv0.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
```

Set the number of pole pairs.
 - RMDX10: 21 (42 poles and 36 slots)
 - R100kv90: 21
 - 8318: 14

```
odrv0.axis0.config.motor.pole_pairs = 21
```

Other initial settings to use:
```
odrv0.axis0.config.calibration_lockin.current = 20
odrv0.axis0.config.motor.current_soft_max = 60
odrv0.axis0.config.motor.current_hard_max = 80
odrv0.axis0.controller.config.vel_limit = 20
odrv0.axis0.config.motor.current_control_bandwidth = 2000
odrv0.axis0.config.encoder_bandwidth = 1500
odrv0.axis0.controller.config.spinout_mechanical_power_threshold = -1000
odrv0.axis0.controller.config.spinout_electrical_power_threshold = 1000
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_closed_loop_control = True
```
Note that `resistance_calib_max_voltage` can't be more than half your bus voltage.

```
odrv0.axis0.config.motor.resistance_calib_max_voltage = 20
```

## Calibration

1. odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
2. odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
3. odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

For troubleshooting the encoder values:
```
odrv0.axis0.requested_state = AxisState.ENCODER_DIR_FIND.error
```
- should return 0.

```
odrv0.axis0.encoder.config.phase_offset
```
- This should print a number, like -326 or 1364.

```
odrv0.axis0..encoder.config.direction
```
- This should print 1 or -1.

To test position control:
```
odrv0.axis0.controller.input_pos = 1
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
```
Or, for velocity:
```
odrv0.axis0.controller.input_vel = 1
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
```
To disable:
```
odrv0.axis0.requested_state = AxisState.IDLE
```

## Useful functions
```
dump_errors(odrv0)
odrv0.save_configuration()
odrv0.clear_errors()
```

To get shadow count:
```
odrv0.onboard_encoder0
```

[Extra info on the API, which isn't in the documentation](https://github.com/odriverobotics/ODrive/blob/master/Firmware/odrive-interface.yaml)

## CAN bus settings

```
odrv0.axis0.config.can.node_id = 0
odrv0.can.config.baud_rate = 100000
```
## Safety
```
axis.config.enable_watchdog = True
axis.config.watchdog_timeout = 5
```

