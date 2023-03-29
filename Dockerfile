FROM tpardi/vtk8.2:latest 
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /apps

RUN apt-get update && apt-get upgrade -y
RUN apt-get install \
    libpcl-dev -y\
    nlohmann-json3-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev -y

COPY . .
RUN mkdir build && cd build && cmake .. && make -j4
