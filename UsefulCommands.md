scripts/shell.sh
scripts/shell.sh rostopic echo /judgement/exchange_markers
scripts/shell.sh rostopic echo /judgement/markers_time
scripts/shell.sh rostopic echo /all_detect_ID
scripts/shell.sh rostopic echo /get_blockinfo
scripts/shell.sh rostopic echo /cmd_vel
scripts/shell.sh rostopic echo /move_base/goal
# 打开rviz，rviz默认关闭
scripts/shell.sh rviz rviz -d /opt/ep_ws/src/navigation/move_base/rviz/teb.rviz

source /opt/ros/noetic/setup.bash
source /devel/setup.bash
roslaunch move_base navigation.launch

source /opt/ros/noetic/setup.bash
rostopic echo /odom | grep position -4