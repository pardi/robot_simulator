FROM tpardi/vtk8.2:latest 
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /apps

COPY . .
RUN mkdir build && cd build && cmake .. && make -j4
