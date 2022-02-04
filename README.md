# Introduction

This repository contains a ROS driver for Blueprint Subsea's Oculus series of multibeam sonars. It has been successfully tested with both the m750d and the m1200d models

# Getting Started

## Dependencies

 * Blueprint Oculus SDK, currently included within the `src` directory. 

## Installation

Clone this repository into the `src` directory of your ROS workspace, and run `catkin_make` from its top directory:

```sh
cd src
git clone --recursive slb-swt@vs-ssh.visualstudio.com:v3/slb-swt/marine-robotics/sonar_oculus
cd ..
catkin_make
```

## Releases

While there are currently no releases, we plan to create a tag for every field experiment.

## Usage

Use the `sonar_oculus.launch` launch file and its arguments to control which processes to run

### Options
 * `static_sonar` - starts a `static_transform_publisher` that publishes a static transform between the `sonar` and local_origin` reference frames (default=false)

### Parameters
 * `sonar_frame` - the name of the sonar reference frame (default: sonar)

### Processses

 * `run_publisher` - sonar publisher - finds and connects to a sonar on the network and starts publishing `OculusPing` messages (default=true)
 * `run_viewer` - start the sonar viewer GUI (default=true)
 * `run_writer` - start a process that subscribes to `OculusPing`  messages and writes them to disk (under `$HOME/.ros`) as PNG files (default=false)
 * `run_ranger` - start a process that extracts range measurements based on the strongest value above a threshold and publishes them as a `sensor_msgs/LaserScan`
 * `run_rqt` - open a dynamic configuration window to set sonar parameter such as gain, range, and frequency (default=true)
 

### Examples

Operating the sonar in an experiment:

```sh
roslaunch sonar_oculus sonar_oculus.launch 
```

Dumping all sonar pings to disk

```sh
roslauch sonar_oculus sonar_oculus.launch run_publisher:=false run_rqt:=false run_writer:=true
# rosbag play bagfile
```

# Contribute

This repository follows the [git flow](https://nvie.com/posts/a-successful-git-branching-model/) branching model. Most development should take place either on the `develop` branch, or on dedicated `feature/<name-of-feature>` branches.
