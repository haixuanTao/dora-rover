export PYLOT_HOME=/home/nvidia/Documents
export PYTHONPATH=/home/nvidia/Documents/dependencies
. $(pwd)/.env/bin/activate

../dora/target/debug/dora-coordinator  --run-dataflow dataflow.yml
export PATH=