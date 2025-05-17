source ~/.bashrc;
source /opt/ros/humble/setup.bash
cd /root/ros2_ws/src
ros2 launch traversability_analysis traversability_analysis_launch.py #> /media/baba/Data/internships/LS2N/intern/SourceCode/src/data/launchTraversability.log
# gdb --args  /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/traversability_analysis/lib/traversability_analysis/traversability_analysis --ros-args -r __node:=traversability_analysis --params-file /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/traversability_analysis/share/traversability_analysis/config/param.yaml #> /media/baba/Data/internships/LS2N/intern/SourceCode/src/data/launchTraversability.log
# valgrind --leak-check=full --track-origins=yes /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/traversability_analysis/lib/traversability_analysis/traversability_analysis --ros-args -r __node:=traversability_analysis --params-file /media/baba/Data/internships/LS2N/intern/SourceCode/src/install/traversability_analysis/share/traversability_analysis/config/param.yaml > /media/baba/Data/internships/LS2N/intern/SourceCode/src/data/launchTraversability.log
bash


