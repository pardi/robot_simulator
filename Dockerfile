FROM tpardi/vtk8.2:latest 
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /apps

RUN apt-get update && apt-get upgrade -y
RUN apt-get install \
    libpcl-dev -y\
    nlohmann-json3-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    git -y

RUN git clone --recurse-submodules https://github.com/pardi/robot_simulator.git robot_sim
RUN mkdir -p robot_sim/build && cd robot_sim/build && cmake .. && make -j4
