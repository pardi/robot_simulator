[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![build](https://github.com/pardi/robot_simulator/actions/workflows/build.yml/badge.svg?event=push)

# Robot simulator based on VTK8.2.

Robot simulator is a standlone simulator written in VTK8.2 library. A sequence of features is already available and exposed to the user such as: 
- load a robot by URDF
- move the robot in a new configuration
- visualise paths
- import point clouds
- draw paths
- draw points
- draw reference frames

The architecture of the code is structured as server/client paradigm and utilises C++14.

The server must be running for the clients to connect to it. The connection type is via TCP/IP and strict messages are used (this makes the software difficult to adopt crossplatform, more on the TODOs section). 


## Requirements: 
This package requires the installation of the following packages. To avoid issue with the lib versioning, please follow 
the order.

RUN apt-get install \


- [vtk == 8.2](https://github.com/pardi/VTK/tree/release/vtk_8.2.0)


## Installation
Install packages:

```
    sudo apt install libpcl-dev nlohmann-json3-dev liburdfdom-dev liburdfdom-headers-dev
```
Install source:
```
    mkdir -p robot_simulator/build
    cd robot_simulator/build
    cmake ..
```

If you want to build the examples, use instead:
    
```
    cmake .. -DCOMPILE_EXAMPLE=ON
```

Then:
```
    make -j 4
    sudo make install
```


## Getting started

TBD
## Sample image

![](https://github.com/pardi/robot_simulator/blob/master/images/panda_movement.gif)

## TODOs
- [ ] move messages to gRPC
