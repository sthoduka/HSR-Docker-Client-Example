# HSR Docker Client Example

This repository contains an example of a client application for the Toyota HSR using Docker.

## Configuration

In ```hsr-docker-client-example/start.sh``` modify the following line as appropriate:
```
export ROS_IP=192.168.1.224 # this is the IP of your machine on the network, not the HSR
```

## Usage

Build: ```sudo docker build -t  hsr-docker-client-example hsr-docker-client-example```

Run: ```sudo docker run --network host --name [your_container_name] hsr-docker-client-example```
