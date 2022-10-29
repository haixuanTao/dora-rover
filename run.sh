source /home/demo/workspace/mapping_ws/devel/setup.bash
roslaunch lidar_localizer ndt_mapping.launch

sleep 5
export PYLOT_HOME=/home/demo/Documents
export PYTHONPATH=/home/demo/Documents/dependencies
. $(pwd)/.env/bin/activate

../dora/target/debug/dora-coordinator  --run-dataflow dataflow.yml
