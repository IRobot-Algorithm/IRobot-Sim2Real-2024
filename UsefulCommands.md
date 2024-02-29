scripts/shell.sh
rostopic echo /judgement/exchange_markers
rostopic echo /judgement/markers_time
rostopic echo /all_detect_ID
rostopic echo /get_blockinfo
scripts/shell.sh rostopic echo /cmd_vel

# 打开rviz，rviz默认关闭
scripts/shell.sh rviz rviz -d /opt/ep_ws/src/navigation/move_base/rviz/teb.rviz