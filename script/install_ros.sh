
### Install ROS

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update -y

sudo apt install ros-noetic-desktop-full -y

source /opt/ros/noetic/setup.sh 

echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc


### Install Velodyne

sudo apt-get install ros-noetic-velodyne -y

mkdir ~/catkin_ws/src/ -p

cd ~/catkin_ws/src/ && git clone https://github.com/ros-drivers/velodyne.git

rosdep install --from-paths src --ignore-src --rosdistro noetic -y

cd ~/catkin_ws/ && catkin_make

### Install mavros

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras -y

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

sudo bash ./install_geographiclib_datasets.sh   

rm ./install_geographiclib_datasets.sh

### Install simple ndt slam

cd ~/catkin_ws/src/ && git clone https://github.com/haixuanTao/simple_ndt_slam.git

git clone https://github.com/catkin/catkin_simple.git

sudo chmod +x ./simple_ndt_slam/assets/scripts/setup_lib.sh

sudo ./simple_ndt_slam/assets/scripts/setup_lib.sh

cd ..

sudo apt-get install python3-catkin-tools ros-sensor-msgs

catkin build -DCMAKE_BUILD_TYPE=Release

echo "source /home/nvidia/catkin_ws/devel/setup.bash" >> ~/.bashrc