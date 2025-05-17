source ~/.bashrc;
source /opt/ros/humble/setup.bash
cd  /root/ros2_ws/src/dataset/;
sleep 1;
ros2 bag play rosbag2_2024_03_07-15_52_36/ --clock;# tall grass area.

bash
ros2 bag play rosbag2_2024_03_07-15_47_48/ --clock;# first terrain
ros2 bag play rosbag2_2024_03_07-15_57_13/ --clock;# hill
cd  /media/baba/Data/internships/
ros2 bag play rosbag2_2024_07_15-11_26_10/ --clock;# Lab2
cd /media/baba/Data/internships/LS2N/intern/others/Backup/Data/Ros2/WithoutFilter/;
ros2 bag play rosbag2_2024_03_01-16_50_05/ --clock;# Lab

ros2 bag play /media/baba/Data/internships/LS2N/intern/others/Data/dataset/outside/rosbag2_2024_07_22-16_29_27 --clock;# first terrain outside with image
ros2 bag play /media/baba/Data/internships/LS2N/intern/others/Data/dataset/outside/rosbag2_2024_07_22-16_22_15 --clock;# first terrain outside no image