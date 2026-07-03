#!/usr/bin/env bash
# Start the guarded robot base on boot (sourced by the_robot.service).
#
# It sources ROS 2 + this workspace, then launches the HC-SR04 collision-guard
# chain. Edit the ros2 launch line if you want a different set of nodes.
set -e

source /opt/ros/humble/setup.bash
source /home/eng-abd/ros2_ws/install/setup.bash

# ROS 2 needs a domain id + a place for its logs when there is no interactive
# shell. Match these to whatever your other machines use.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 launch the_robot guarded_base.launch.py
