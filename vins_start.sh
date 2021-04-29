#!/bin/bash
#测试sh程序
rostopic list
a="hello world!"
num=2
echo "a is : $a num is : ${num}nd"
#gnome-terminal --window --tab --tab
#{
#	gnome-terminal -t "XXD_ros" -- bash -c "rostopic list;echo "ros";exec bash"
#}&
#sleep 2s
{
	echo "a is : $a num is : ${num}nd"
	gnome-terminal --tab  -- bash -c  "cd ~/robot/m_robot/m_robot;source devel/setup.bash ; rosrun vins vins_node /home/q/robot/m_robot/m_robot/src/VINS/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml;exec bash"
}

{
	echo "a is : $a num is : ${num}nd"
	gnome-terminal --tab  -- bash -c  "cd ~/robot/m_robot/m_robot;source devel/setup.bash ; rosrun loop_fusion loop_fusion_node src/VINS/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml;exec bash"
}

{
	echo "a is : $a num is : ${num}nd"
	gnome-terminal --tab  -- bash -c  "cd ~/robot/m_robot/m_robot;source devel/setup.bash ;roslaunch vins vins_rviz.launch;exec bash"
}

sleep 1s
{
	#gnome-terminal -t "XXD_ros" -- bash -c "rostopic list;echo "ros";exec bash"
	gnome-terminal  -t "XXD_ros" -- bash -c "echo "cd";cd /media/q/neu/dataset/EuRoC; rosbag play MH_01_easy.bag;exec bash"
}&
