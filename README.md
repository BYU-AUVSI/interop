# BYU Interop Client [![Build Status](https://travis-ci.org/BYU-AUVSI/interop.svg?branch=master)](https://travis-ci.org/BYU-AUVSI/interop)
This ROS package interfaces BYU's systems with the judge's server for the AUVSI-SUAS competition.

## Setup
There are two ways to setup the interop client: as a ROS workspace, or as a docker container.

### ROS Workspace
The repository is formatted as a ROS package named "auvsi_client". This means that you can simply include the interop repository in whatever workspace you're currently using. If you want to create a new workspace for the interop client, a script has been provided which creates a workspace and downloads needed dependencies. It can be ran like so:
```
cd [downloaded repo directory]/interop
./setup.sh
```
To run from the workspace, use the following commands:
```
export SERVER=192.168.?.? # the address of the judge's server
export SERVER_PORT=80 # the port of the judge's server
source devel/setup.bash
rosrun auvsi_client client.py
```

### Docker
If you have docker installed, you can run the latest version of the interop client with the following command:
```
docker run -it --net="host" mcrossen/interop
```
The above command only works if all other needed servers, nodes, and services are on the local machine. To specify additional details such as the judges server address, the ROS master addresss, or the judges port, use the following command:
```
docker run -it --net="host" -e MASTER=192.168.?? -e SERVER=192.168.?? -e PORT=80 mcrossen/interop
```

## Notes
Here are some other useful commands and hints:

### Docker installation
To install docker, follow the [official guide](https://docs.docker.com/engine/installation/linux/ubuntu/#install-using-the-repository).

### Judges Server
To run the judges server on a computer, first install docker. After installing docker, run the following command:
```
docker run -it -p 80:80 auvsisuas/interop-server
```
