#!/bin/bash

source ~/.bashrc
find_pkg=$(rospack find point_cloud_fusion)
find_ws=$find_pkg/../..
gnome-terminal --title="livox_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud repub_3dRebuild_tp_4bag.launch"
gnome-terminal --title="livox_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch point_cloud_fusion point_cloud_sync_fusion_4pc.launch cloud1_tp:=/livox_1/lidar cloud2_tp:=/livox_2/lidar cloud3_tp:=/livox_3/lidar cloud4_tp:=/livox_5/lidar"