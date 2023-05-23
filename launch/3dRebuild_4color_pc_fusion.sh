#!/bin/bash

source ~/.bashrc
find_pkg=$(rospack find point_cloud_fusion)
find_pkg_color=$(rospack find colored_pointcloud)
find_ws=$find_pkg/../..
# gnome-terminal --title="livox_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud repub_3dRebuild_tp_4bag.launch"
gnome-terminal --title="color_pc" -- bash -c "bash $find_pkg_color/launch/3dRebuil_color_pc_4bags.sh"
sleep 1s
gnome-terminal --title="livox_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch point_cloud_fusion point_cloud_sync_fusion_4pc_color.launch cloud1_tp:=/scene1/colored_cloud_toshow cloud2_tp:=/scene2/colored_cloud_toshow cloud3_tp:=/scene3/colored_cloud_toshow cloud4_tp:=/scene4/colored_cloud_toshow"