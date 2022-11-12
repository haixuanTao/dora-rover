virtualenv .env

. $(pwd)/.env/bin/activate

# Install dora Python node API 
pip install maturin
cd ../dora/apis/python/node
python3 -m maturin develop
cd ../../../../dora-rover

# install Dependencies
pip install --upgrade pip
pip install -r requirements.txt

## Install Pytorch

sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev

wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
pip install torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl

## Install torchvision

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch 0.13.0 https://github.com/pytorch/vision torchvision   # see below for version of torchvision to download
cd torchvision
export BUILD_VERSION=0.13.0  # where 0.x.0 is the torchvision version  
python3 setup.py install --user
cd ../

## Install Frenet Trajectory

cd ../
mkdir dependencies -p
export 
export PYLOT_HOME=/home/demo/Document
sgit clone https://github.com/erdos-project/frenet_optimal_trajectory_planner.git
cd frenet_optimal_trajectory_planner/
bash build.sh

export PYTHONPATH=$PYTHONPATH:/home/demo/Documents/dependencies
echo "export PYTHONPATH=/home/demo/Documents/dependencies" >> ~/.bashrc
echo "export PYLOT_PATH=/home/demo/Documents" >> ~/.bashrc