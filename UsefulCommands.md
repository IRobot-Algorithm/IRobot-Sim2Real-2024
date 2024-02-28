rostopic echo /judgement/exchange_markers
rostopic echo /judgement/markers_time
rostopic echo /all_detect_ID
rostopic echo /get_blockinfo

# 打开rviz，rviz默认关闭
rviz rviz -d /opt/ep_ws/src/navigation/move_base/rviz/teb.rviz