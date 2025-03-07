# Anscer Robotics Interview

The task was to create a ROS package with two nodes

- One for collecting, visualizing and storing robot's trajectory in format specified by the user and specific time duration mentioned by the user

- Second was from reading the data from the file, transforming to odom frame and then displaying.

I provide this repository as solution to the above problem

## Usage

This repository is docker container, and all the necessary packages are inside src folder

```
# unzip the provided folder, first thing populate submodules

git submodule update --init --recursive

# After that build the docker container
make build && make up

# To go inside the container
make exec

```

## anscer_trajectory_utils

This is the main package for reader and saver nodes, the folder structure is as per the guidelines of the ros

```
.
├── config <-- contains config params for reader and saver launch files
├── include
│   └── anscer_trajectory_utils
├── launch <-- contains launch files for reader and saver launch files
└── src
```

### Usage

- Setting up the package

```
# once inside the docker container
cd ws_anscer
colcon build --symling-install
source install/setup.zsh
```

#### Saver Node

- Saver node, this node relies on the topic name that the user wants to record by default it is set to `/odom`, you can change this in `config/trajectory_saver_node.yaml`, and keep `use_sim_time` to `false` if using it on real robot.

```
# After the above steps it is pretty simple
ros2 launch anscer_trajectory_utils trajectory_saver_node.launch.py
```

This will spawn the service `/save_trajectory` as well, then to save_trajectory

```
# If you are familiar with foxglove then its easy as well, but with cli
ros2 service call /anscer_msgs/srv/SaveTrajectory "{filename: 'output.json', duration: 10}"
```

- NOTE: filename should be a string, and extensions can be json,csv,yaml and duration are in seconds

#### Reader Node

Reader node, the config file of this node contains `filepath`: aboslute path to file, `base_frame_id` default is `base_footprint` and `map_frame_id`: default is `/odom`, if filename is not valid or it is not there in location then the node will return error. It is advised to user to take a look into `config/trajectory_reader_params.yaml`

```
# To call reader node

ros2 launch anscer_trajectory_utils anscer_reader_node.launch.py
```

- This will publish the trajectory transformed in odom frame on the topic once and you can visualize in RViZ or Foxglove

## Maintainer

This is mainted by Dhagash Desai, in case of any doubts contact `desaidhagash@gmail.com`
