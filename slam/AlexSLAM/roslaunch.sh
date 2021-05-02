export ROS_MASTER_URI=http://astrio-Latitude-7390:11311
source ~/Desktop/slam/devel/setup.bash
cd ~/Desktop/slam/
catkin_make
roslaunch rplidar_ros rplidar.launch
