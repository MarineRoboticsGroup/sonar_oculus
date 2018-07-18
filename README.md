# Introduction

This repository contains a ROS driver for Blueprint Subsea's Oculus series of multibeam sonars. It has been successfully tested with both the m750d and the m1200d models

# Getting Started

## Dependencies

## Installation

Clone this repository into the `src` directory of your ROS workspace, and run `catkin_make` from its top directory:

```sh
cd src
git clone --recursive ssh://slb-swt@vs-ssh.visualstudio.com:22/SDR/_ssh/sonar_oculus
cd ..
catkin_make
```

## Releases

While there are currently no releases, we plan to create a tag for every field experiment.


## Usage

Use the `sonar_oculus.launch` launch file and its arguments to control which processes to run:
 * `publisher` - sonar publisher - finds and connects to a sonar on the network and starts publishing `OculusPing` messages (default=true)
 * `viewer` - start the sonar viewer GUI (default=true)
 * `writer` - start a process that subscribes to `OculusPing`  messages and writes them to disk (under `$HOME/.ros`) as PNG files (default=false)
 * `rqt` - open a dynamic configuration window to set sonar parameter such as gain, range, and frequency (default=true)
 

### Examples

Operating the sonar in an experiment:

```sh
roslaunch sonar_oculus sonar_oculus.launch 
```

Dumping all sonar pings to disk

```sh
roslauch sonar_oculus sonar_oculus.launch publisher:=false rqt:=false writer:=true
# rosbag play bagfile
```

# Contribute

This repository follows the [git flow](https://nvie.com/posts/a-successful-git-branching-model/) branching model. Most development should take place either on the `develop` branch, or on dedicated `feature/<name-of-feature>` branches.
