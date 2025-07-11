#!/bin/bash

echo "Thesis bagfile recorder REAL EXPERIMENTS"
read -p "Bagfile name: " name

echo "Start recording " $name "!"

rosbag record -O $name.bag /akf/odom /akf/state_x /akf/state_y \
/point_lio/odom /point_lio/eig /point_lio/eig2 /point_lio/n_points /point_lio/trace \
/mavros/global_position/local /mavros/local_position/pose \
/ov_msckf/odomimu /ov_msckf/eig \
/tf /tf_static /inject_blur/state
