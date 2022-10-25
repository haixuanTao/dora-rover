virtualenv .env


. $(pwd)/.env/bin/activate

# Dev dependencies
pip install maturin
cd ../dora/apis/python/node
maturin develop
cd ../../../../dora_ros_bridge

# Dependencies
pip install --upgrade pip
pip install -r requirements.txt

../dora/target/debug/dora-coordinator  --run-dataflow dataflow.yml
