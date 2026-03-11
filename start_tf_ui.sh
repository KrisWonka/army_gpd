#!/usr/bin/env zsh

set -e

# Usage:
#   ./restart_tf_ui.sh [enable_points_base] [single_shot_cloud_topic]
# Example:
#   ./restart_tf_ui.sh false /rgbd_cam/depth/points_base_pc

ENABLE_POINTS_BASE="${1:-false}"
SINGLE_SHOT_CLOUD_TOPIC="${2:-/rgbd_cam/depth/points_base_pc}"

cleanup() {
  pkill -f "roslaunch jetarm_ui gpd_viz_gazebo.launch" || true
  pkill -f "gpd_viz_rviz" || true
  pkill -f "rviz" || true
}

trap cleanup EXIT INT TERM

pkill -f "roslaunch jetarm_ui tf_calibration_control_ui.launch" || true
pkill -f "roslaunch jetarm_ui gpd_viz_gazebo.launch" || true
pkill -f "gpd_viz_rviz" || true
pkill -f "rviz" || true
pkill -f "roslaunch jetarm_peripherals camera.launch" || true
pkill -f roscore || true
pkill -f rosmaster || true
sleep 1

unset ROS_HOSTNAME
export ROS_IP=10.42.0.222
export ROS_MASTER_URI=http://10.42.0.222:11311
export PYTHONNOUSERSITE=1
unset PYTHONPATH

source /opt/ros/melodic/setup.zsh
cd /home/hiwonder/jetarm
source devel/setup.zsh

wait_for_master() {
  local i
  for i in {1..80}; do
    if rosparam get /rosversion >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

wait_for_run_id() {
  local i
  for i in {1..80}; do
    if rosparam get /run_id >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# Start master first to avoid roslaunch run_id race.
roscore >/tmp/roscore_tf_ui.log 2>&1 &
wait_for_master || {
  echo "ERROR: roscore not ready"
  exit 1
}

# Start RViz visualization stack in background.
roslaunch jetarm_ui gpd_viz_gazebo.launch \
  start_gazebo:=false \
  start_gpd_grasp:=false &

wait_for_run_id || {
  echo "ERROR: /run_id not ready after gpd_viz_gazebo.launch"
  exit 1
}

roslaunch jetarm_ui tf_calibration_control_ui.launch \
  enable_points_base:="${ENABLE_POINTS_BASE}" \
  single_shot_cloud_topic:="${SINGLE_SHOT_CLOUD_TOPIC}"
