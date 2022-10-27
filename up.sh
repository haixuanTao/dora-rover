roslaunch velodyne_pointcloud VLP16_points.launch &
sleep 2
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:57600 &
sleep 2
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c GUIDED
