#!/bin/bash
# organize_desktop.sh — 바탕화면 정리 스크립트

DESKTOP='/mnt/c/Users/USER/Desktop'
SCRIPTS='/mnt/c/Users/USER/Desktop/ARGOS/scripts'
SCREENSHOTS='/mnt/c/Users/USER/Desktop/ARGOS/learning-log/screenshots/week03'

# 폴더 생성
mkdir -p "$SCRIPTS/gazebo" "$SCRIPTS/nav2" "$SCRIPTS/slam" \
         "$SCRIPTS/diagnostics" "$SCRIPTS/rviz" "$SCRIPTS/misc"
mkdir -p "$SCREENSHOTS"

move_file() {
  local src="$DESKTOP/$1"
  local dst="$2/$1"
  if [ -f "$src" ]; then
    mv "$src" "$dst" && echo "  ✓ $1"
  fi
}

echo "[gazebo]"
move_file start_gazebo.sh "$SCRIPTS/gazebo"
move_file test_gazebo_direct.sh "$SCRIPTS/gazebo"

echo "[nav2]"
for f in launch_nav2.sh launch_nav2_clean.sh kill_nav2.sh restart_bridge.sh \
          nav_goal_far.sh nav_goal_full.sh nav_test_move.sh send_nav_goal.sh; do
  move_file "$f" "$SCRIPTS/nav2"
done

echo "[slam]"
for f in activate_slam.sh launch_slam.sh check_slam.sh check_slam_proc.sh check_slam_status.sh; do
  move_file "$f" "$SCRIPTS/slam"
done

echo "[diagnostics]"
for f in check_clock.sh check_display.sh check_display2.sh check_gz_log.sh \
          check_nav2.sh check_nav2_full.sh check_nav2_lifecycle.sh \
          check_nav_status.sh check_robot_motion.sh kill_all_ros.sh; do
  move_file "$f" "$SCRIPTS/diagnostics"
done

echo "[rviz]"
for f in start_rviz2.sh capture_rviz.sh bring_rviz.ps1 capture_by_handle.ps1 \
          capture_rviz.ps1 find_all_windows.ps1 focus_rviz.ps1 \
          list_windows.ps1 screenshot.ps1 screenshot_all.ps1; do
  move_file "$f" "$SCRIPTS/rviz"
done

echo "[misc]"
move_file manual_save_conv.sh "$SCRIPTS/misc"
move_file save_conv.sh "$SCRIPTS/misc"

echo "[screenshots]"
for f in nav2_goal_attempt.png nav2_pose_set.png nav2_rviz.png rviz_lidar.png \
          rviz_only.png rviz_screenshot.png rviz_screenshot2.png \
          screen_capture.png screen_full.png; do
  move_file "$f" "$SCREENSHOTS"
done

echo ""
echo "정리 완료!"
