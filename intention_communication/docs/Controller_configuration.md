# ropod_teleop

## Summary

A joypad for the ROPOD platform exposed as a ROS node.

## Supported key combinations

* Controller initialisation: `L1 + R1 + X`
* Controller stopping: `L1 + R1 + S`
* Linear motion: `R1 + left axis`
* Rotation: `R1 + right axis`
* Docking: `R1 + L1 + O`
* Undocking: `R1 + L1 + Tri`
* Event registration for data annotation: `L1 + S`
* Individual smart wheel motion:
    * Wheel 1: `R1 + Tri + cross`
    * Wheel 2: `R1 + O + cross`
    * Wheel 3: `R1 + X + cross`
    * Wheel 4: `R1 + Sq + cross`

## Launch file parameters

The following parameters may be passed when launching the teleop node:

* `joy_topic`: Topic for joypad events (default `/joy`)
* `number_of_smart_wheels`: Number of smart wheels on a given ropod (default `4`)
* `base_vel_topic`: Topic for base velocity commands (default `/cmd_vel`)

