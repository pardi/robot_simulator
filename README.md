[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![build](https://github.com/pardi/robot_simulator/actions/workflows/build.yml/badge.svg?event=push)

# Robot simulator based on VTK8.2.

The architecture of the code is structured as server/client. The server runs once at the beginning and clients connects to it via a TCP/IP socket.


C++14 standard is used in this library.

## Sample image

![](https://github.com/pardi/robot_simulator/blob/master/images/panda_movement.gif)
