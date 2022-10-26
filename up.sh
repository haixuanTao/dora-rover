roslaunch velodyne_pointcloud VLP16_points.launch &
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:57600 &
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c GUIDED