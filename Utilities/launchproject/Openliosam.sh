source ~/.bashrc;
source /opt/ros/humble/setup.bash
ros2 launch lio_sam run.launch.py
bash

# gdb --args /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization --ros-args -r __node:=lio_sam_mapOptimization --params-file /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/lio_sam/share/lio_sam/config/params.yaml
