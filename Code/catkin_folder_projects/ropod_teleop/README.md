# ropod_teleop

## Summary

A joypad for the ROPOD platform exposed as a ROS node.

## Supported key combinations

* Controller initialisation: `LB + RB + A`
* Controller stopping: `LB + RB + X`
* Linear motion: `RB + left axis`
* Rotation: `RB + right axis`
* Docking: `RB + LB + B`
* Undocking: `RB + LB + Y`
* Event registration for data annotation: `LB + X`
* Individual smart wheel motion:
    * Wheel 1: `RB + Y + cross Y`
    * Wheel 2: `RB + B + cross Y`
    * Wheel 3: `RB + A + cross Y`
    * Wheel 4: `RB + X + cross Y`

## Launch file parameters

The following parameters may be passed when launching the teleop node:

* `joy_topic`: Topic for joypad events (default `/joy`)
* `number_of_smart_wheels`: Number of smart wheels on a given ropod (default `4`)
* `base_vel_topic`: Topic for base velocity commands (default `/cmd_vel`)
* `smart_wheel_command_topic`: Topic for individual smart wheel velocity commands (default `/smart_wheel/command`)
* `docking_command_topic`: Topic for docking/undocking commands (default `/ropod/ropod_low_level_control/cmd_dock`)
* `event_topic`: Topic for events (default `/ropod/event`)
* `move_base_cancel_topic`: Topic for cancelling move base commands (default `/move_base/cancel`)
* `max_linear_vel`: Maximum base linear velocity in m/s (default `0.1`)
* `max_angular_vel`: Maximum base angular velocity in rad/s (default `0.1`)
* `bringup_launcher`: Absolute path to a bringup launcher for the ropod (default `''`)

## Dependencies

* `rospy`
* `roslaunch`
* `joy`
* `sensor_msgs`
* `geometry_msgs`
* `actionlib_msgs`
* `ropod_ros_msgs`
