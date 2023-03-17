FROM ubuntu:20.04 AS ci_minimal
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /app

RUN apt-get update && apt-get upgrade -y
RUN apt install build-essential cmake libglu1-mesa-dev -y
RUN apt install freeglut3-dev mesa-common-dev wget libpcl-dev -y
RUN wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz
RUN tar -xvzf VTK-8.2.0.tar.gz
RUN mdkir VTK-8.2.0/build && cd VTK-8.2.0/build && cmake .. && make -j8 && make install

