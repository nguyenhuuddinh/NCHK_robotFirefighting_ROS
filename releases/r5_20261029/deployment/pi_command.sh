umask 077
source /opt/ros/humble/setup.bash &&
source /home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install/setup.bash &&
source /home/pi/r5_staging/0c54d9b79765/ros/ros2_ws/install/local_setup.bash &&
R5_PI_OUT="/home/pi/qa_logs/r5_automatic_$(date +%Y%m%d_%H%M%S)" &&
printf 'LOG_PI=%s\n' "$R5_PI_OUT" &&
python3 /home/pi/r5_staging/0c54d9b79765/ros/scripts/r5_pi_session.py --run-observe --out "$R5_PI_OUT"
