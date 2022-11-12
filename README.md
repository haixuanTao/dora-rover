# Python Dataflow Example

This examples shows how to create and connect dora operators and custom nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- a webcam node, that connects to your webcam and feed the dataflow with webcam frame as jpeg compressed bytearray.
- an object detection node, that apply Yolo v5 on the webcam image. The model is imported from Pytorch Hub. The output is the bouding box of each object detected, the confidence and the class. You can have more info here: https://pytorch.org/hub/ultralytics_yolov5/
- a window plotting node, that will retrieve the webcam image and the Yolov5 bounding box and join the two together.

## Getting started

```bash
cargo run --example python-dataflow 
```

## Installation

To install, you should run the `install.sh` script.

```bash
install.sh
```

## Run the dataflow as a standalone

- Start the `dora-coordinator`, passing the paths to the dataflow file and the `dora-runtime` as arguments:

```
../../target/release/dora-coordinator --run-dataflow dataflow.yml ../../target/release/dora-runtime
```

## Installation of Pytorch and Torchvision

See: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048


# In dora_ro
# Open dora-coordinator
. /home/demo/Documents/dora_ros_bridge/.env/bin/activate
../dora/target/debug/dora-coordinator &
# open iceoryx
find ../dora/target -type f -wholename "*/iceoryx-install/bin/iox-roudi" -exec {} \; &

# arm
rosrun mavros mavsafety arm

# SYS configuration
rosrun mavros mavparam set SYSID_MYGCS 1

# Start dataflow
../dora/target/debug/dora-cli start graphs/dataflow.yml

# STOP override

rostopic pub /mavros//override mavros_msgs/OverrideRCIn "{channels: [1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

# or 

rosrun mavros mavsys mode -c HOLD
