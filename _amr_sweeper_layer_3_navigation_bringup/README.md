# amr_sweeper_layer_3_navigation_bringup

`ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py`

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_waypoint_follower`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`

## Purpose
This package is the main entrypoint for the AMR Sweeper navigation layer.

## Main Launch File
`launch/amr_sweeper_layer_3_navigation_bringup.launch.py`

## Available Launch Files
- `amr_sweeper_layer_3_navigation_bringup.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_localization`: default `true`
- `use_navigation`: default `true`

## Overview
`amr_sweeper_layer_3_navigation_bringup` starts the localization package and the Nav2/waypoint package together so the AMR Sweeper can estimate its pose and generate motion commands. It is the top-layer bringup for the real robot navigation stack.

## Notes
- Use this package after the required layer 1 and layer 2 packages are already available.
- The navigation commands produced here flow down through layer 2 and into the layer 1 drive interfaces.
