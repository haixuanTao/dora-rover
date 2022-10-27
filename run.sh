source /home/demo/workspace/mapping_ws/devel/setup.bash
roslaunch lidar_localizer ndt_mapping.launch

sleep 5


. $(pwd)/.env/bin/activate

../dora/target/debug/dora-coordinator  --run-dataflow dataflow.yml
